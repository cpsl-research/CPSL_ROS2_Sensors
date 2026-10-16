"""Start-gate decision table (pure logic, no ROS graph, no h5py)."""
import json

from dataset_generator import start_gate as sg


def test_all_seen_opens():
    assert sg.evaluate(['a', 'b'], {'a', 'b'}, 0.1, 30.0, False) == \
        (sg.OPEN, [])


def test_waiting_names_the_silent_stream_in_order():
    assert sg.evaluate(['a', 'b', 'c'], {'b'}, 1.0, 30.0, False) == \
        (sg.WAITING, ['a', 'c'])


def test_timeout_aborts_with_missing():
    assert sg.evaluate(['a', 'b'], {'a'}, 30.0, 30.0, False) == \
        (sg.ABORT, ['b'])
    assert sg.evaluate(['a', 'b'], {'a'}, 29.9, 30.0, False)[0] == sg.WAITING


def test_allow_missing_opens_at_timeout_and_reports():
    assert sg.evaluate(['a', 'b'], {'a'}, 30.0, 30.0, True) == \
        (sg.OPEN, ['b'])
    # not before the timeout
    assert sg.evaluate(['a', 'b'], {'a'}, 5.0, 30.0, True)[0] == sg.WAITING


def test_disabled_streams_are_not_in_enabled():
    # the caller passes only enabled streams: a silent disabled one is moot
    assert sg.evaluate(['a'], {'a'}, 0.0, 30.0, False) == (sg.OPEN, [])
    assert sg.evaluate([], set(), 0.0, 30.0, False) == (sg.OPEN, [])


def test_timeout_zero_waits_forever():
    assert sg.evaluate(['a'], set(), 10 ** 9, 0.0, False) == \
        (sg.WAITING, ['a'])
    assert sg.evaluate(['a'], set(), 10 ** 9, 0, True)[0] == sg.WAITING


def test_log_line_and_json_are_stable():
    assert sg.gate_line(sg.WAITING, ['t1', 't2'], 1.0) == \
        'GATE waiting missing=t1,t2'
    assert sg.gate_line(sg.ABORT, ['t1'], 30.0) == 'GATE abort missing=t1'
    assert sg.gate_line(sg.OPEN, [], 2.5) == 'GATE open wait_s=2.50'
    d = json.loads(sg.gate_json(sg.WAITING, ['a'], ['t'], 1.23456, 30, False))
    assert d == {'state': 'waiting', 'missing': ['a'],
                 'missing_topics': ['t'], 'wait_s': 1.235,
                 'timeout_s': 30.0, 'allow_missing': False}

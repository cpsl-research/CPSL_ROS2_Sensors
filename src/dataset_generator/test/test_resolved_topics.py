"""Resolved-topic / namespace attrs (no ROS graph, no hardware).

Calls the recorder's methods on a stand-in `self`, so rclpy need not run a node.
These test the code, they are not a capture result.
"""
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('h5py')

from dataset_generator.dataset_recorder import DatasetRecorder  # noqa: E402

NSP = '/cpsl_ugv_1'


def fake():
    def resolve(t):
        return t if t.startswith('/') else NSP + '/' + t
    me = NS(resolve_topic_name=resolve, _pending=[], cfgs={},
            _p={'capture_mode': 'all', 'sample_rate_hz': 1.0},
            get_fully_qualified_name=lambda: NSP + '/dataset_generator',
            get_namespace=lambda: NSP, subs=[], seq={}, dropped={},
            cfg_streams={}, attrs={})
    me._resolved = lambda t: DatasetRecorder._resolved(me, t)
    me._mode = lambda cfg: 'all'
    me._qos = lambda cfg, conv: ('reliable', 10, None)
    me._set_attrs = lambda n, a: me.attrs.setdefault(n, {}).update(a)
    return me


def test_resolved_empty_stays_empty():
    assert fake()._resolved('') == ''


def test_root_namespace():
    a = DatasetRecorder._root_attrs(fake())
    assert a['namespace'] == NSP


def test_stream_resolved_topic_keeps_topic():
    me = fake()
    me.meta_pairs = []
    me.create_subscription = lambda *a, **k: object()
    me.get_logger = lambda: NS(info=lambda *a: None)
    cfg = {'type': 'nav_msgs/msg/Odometry', 'topic': 'radar_0/foo'}
    DatasetRecorder._add_stream(me, 's', cfg)
    attrs = me._pending[0][1]
    assert attrs['topic'] == 'radar_0/foo'
    assert attrs['resolved_topic'] == NSP + '/radar_0/foo'


def test_absolute_topic_unchanged():
    assert fake()._resolved('/abs/t') == '/abs/t'


def test_radar_cfg_resolved():
    me = fake()
    me.create_subscription = lambda *a, **k: object()
    me.get_logger = lambda: NS(info=lambda *a: None, warning=lambda *a: None)
    me._on_cfg_path = lambda *a: None
    DatasetRecorder._add_cfg_sub(
        me, 'radar_0', {'topic': 'radar_0/adc_data_cube'})
    assert me.attrs['radar_0']['radar_cfg_topic'] == 'radar_0/radar_config_path'
    assert (me.attrs['radar_0']['radar_cfg_resolved_topic']
            == NSP + '/radar_0/radar_config_path')

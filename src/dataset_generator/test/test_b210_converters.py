"""B210 converters + recorder-side storage, with synthetic messages (no ROS, no radio)."""
import array
import time
from types import SimpleNamespace as NS

from dataset_generator import converters as cv
from dataset_generator.hdf5_writer import CaptureFile
import numpy as np
import pytest

h5py = pytest.importorskip('h5py')

NS_PER_BURST = 56000


def _burst(i, iq=True, n=NS_PER_BURST):
    vals = ((np.arange(2 * n, dtype=np.int64) + i * 7) % 65536 - 32768).astype(np.int16)
    return NS(header=NS(stamp=NS(sec=100 + i, nanosec=5), frame_id=''),
              index=i, sweep=i // 10, step=i % 10, sample_offset=i * n, num_samples=n,
              usrp_ticks=1000 * i, usrp_time_s=0.001 * i, tx_freq_hz=1e9, rx_freq_hz=1.1e9,
              lo_offset_hz=2e6, tx_gain_db=0.0, rx_gain_db=30.0, valid=True, error_flags=0,
              iq=array.array('h', vals.tobytes()) if iq else array.array('h'))


def _open(path):
    cap = CaptureFile(str(path), {})
    cv_ = cv.get_converter('sar_b210_msgs/SarBurst')
    cap.add_stream('b', {'msg_type': 'sar_b210_msgs/msg/SarBurst'}, keep_stamps=False)
    return cap, cv_


def test_registered_reliable():
    assert cv.get_converter('sar_b210_msgs/msg/SarBurst').default_qos == 'reliable'
    assert cv.get_converter('sar_b210_msgs/SarClockSync').default_qos == 'reliable'


def test_burst_roundtrip(tmp_path):
    cap, conv = _open(tmp_path / 'a.h5')
    msgs = [_burst(i, n=64) for i in range(5)]
    for m in msgs:
        s = conv.fn(m, {})
        cap.append('b', s.header_ns, 1, s.fields, s.ragged)
    cap.close()
    with h5py.File(tmp_path / 'a.h5') as f:
        g = f['b']
        assert g['iq'].shape == (5, 128) and g['iq'].dtype == np.int16
        assert (g['iq'][3] == np.frombuffer(msgs[3].iq, np.int16)).all()
        assert g['index'].dtype == np.uint64 and list(g['index'][:]) == list(range(5))
        assert g['valid'].dtype == np.bool_ and g['stamp_header_ns'][2] == 102 * 10**9 + 5
        assert g['sample_offset'][4] == 4 * 64


def test_burst_without_iq_is_metadata_only(tmp_path):
    cap, conv = _open(tmp_path / 'b.h5')
    for i in range(3):
        s = conv.fn(_burst(i, iq=False), {})
        cap.append('b', s.header_ns, 1, s.fields, s.ragged)
    cap.close()
    with h5py.File(tmp_path / 'b.h5') as f:
        assert 'iq' not in f['b'] and f['b']['index'].shape == (3,)


def test_bad_iq_length_and_bytes_input():
    m = _burst(0, n=8)
    m.iq = array.array('h', [0] * 5)
    with pytest.raises(ValueError):
        cv.get_converter('sar_b210_msgs/SarBurst').fn(m, {})
    m.iq = np.arange(16, dtype=np.int16).tobytes()
    s = cv.get_converter('sar_b210_msgs/SarBurst').fn(m, {})
    assert s.fields['iq'].shape == (16,)


def test_clock_sync(tmp_path):
    conv = cv.get_converter('sar_b210_msgs/msg/SarClockSync')
    s = conv.fn(NS(usrp_ticks=1, host_monotonic_ns=2, host_realtime_ns=3, round_trip_ns=4), {})
    assert s.header_ns == -1 and s.fields['data'].dtype == np.int64
    assert list(s.fields['data']) == [1, 2, 3, 4]


def test_throughput(tmp_path):
    """Synthetic rate check (not a hardware result): 650 bursts of 112000 int16."""
    cap, conv = _open(tmp_path / 't.h5')
    msgs = [_burst(i) for i in range(4)]
    t0 = time.perf_counter()
    for i in range(650):
        s = conv.fn(msgs[i % 4], {})
        cap.append('b', s.header_ns, 1, s.fields, s.ragged)
    cap.close()
    dt = time.perf_counter() - t0
    mbs = 650 * 224000 / dt / 1e6
    print('throughput %.1f MB/s (%.0f bursts/s)' % (mbs, 650 / dt))
    assert mbs > 14.6   # the live rate; real margin is far larger

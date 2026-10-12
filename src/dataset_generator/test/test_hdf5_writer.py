"""Writer/converter checks with fake messages (no ROS graph, no hardware).

These test the code, they are not a capture result.
"""
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

h5py = pytest.importorskip('h5py')

from dataset_generator import converters as cv  # noqa: E402
from dataset_generator.stream_config import parse_streams  # noqa: E402
from dataset_generator.hdf5_to_legacy import export  # noqa: E402
from dataset_generator.hdf5_writer import CaptureFile  # noqa: E402


def hdr(sec, nsec, frame='f'):
    return NS(stamp=NS(sec=sec, nanosec=nsec), frame_id=frame)


def adc(i, dims=(1, 8, 4)):
    n = int(np.prod(dims))
    return NS(header=hdr(10, i * 1000),
              layout=NS(dim=[NS(label=l, size=s, stride=0)
                             for l, s in zip(('rx', 'sample', 'chirp'), dims)]),
              real_data=list(range(n)), imag_data=[-x for x in range(n)])


def meta(frame_index, i, nchirps=4):
    z = [0] * nchirps
    return NS(header=hdr(10, i * 1000), frame_index=frame_index,
              record_valid=[True] * nchirps, adc_complete=[True] * nchirps,
              flags=z, global_chirp_idx=z, run_idx=z, ts_ticks=z, sat_slices=z,
              sat_ref_lag=z, sat_slices_aligned=z, has_time=[True] * nchirps,
              time_interpolated=[False] * nchirps,
              t_s=[0.1] * nchirps, prev_tail_sat=[-1, -1], run_known=True,
              run_idx_frame=3, records_valid=nchirps, records_invalid=0,
              records_other_run=0)


def cloud(pts):
    arr = np.asarray(pts, np.float32)
    fields = [NS(name=n, offset=4 * k, datatype=7, count=1)
              for k, n in enumerate(('x', 'y', 'z', 'vel'))]
    return NS(header=hdr(1, 0), fields=fields, width=len(arr), height=1,
              point_step=16, is_bigendian=False, data=arr.tobytes())


def put(cap, name, msg, conv, recv, opts=None):
    s = conv.fn(msg, opts or {})
    cap.append(name, s.header_ns, recv, s.fields, s.ragged)


def test_roundtrip_all_types(tmp_path):
    path = tmp_path / 'c.h5'
    cap = CaptureFile(path, {'capture_mode': 'all'}, flush_period_s=0.0)
    cube_c = cv.get_converter('raw_radar_msgs/ADCDataCube')
    meta_c = cv.get_converter('raw_radar_msgs/msg/ADCChirpMeta')
    pc_c = cv.get_converter('sensor_msgs/msg/PointCloud2')
    cap.add_stream('radar_adc', {'topic': 't'}, keep_stamps=True)
    cap.add_stream('adc_meta', keep_stamps=True)
    cap.add_stream('pc')
    for i in range(5):
        put(cap, 'radar_adc', adc(i), cube_c, 100 + i)
        put(cap, 'adc_meta', meta(i, i), meta_c, 101 + i)
    # frame 5 published but its cube was lost
    put(cap, 'adc_meta', meta(5, 5), meta_c, 106)
    put(cap, 'pc', cloud([[1, 2, 3, 4], [np.nan, 0, 0, 0]]), pc_c, 1)
    put(cap, 'pc', cloud([[1, 2, 3, 4]] * 3), pc_c, 2)
    res = cap.cross_check('adc_meta', 'radar_adc')
    assert res == {'n_without_meta': 0, 'n_without_cube': 1,
                   'published': 6, 'recorded': 5, 'gap': 1}
    cap.close()

    with h5py.File(path, 'r') as f:
        assert f.attrs['closed_cleanly'] and f.attrs['format_version'] == 2
        d = f['radar_adc/data']
        assert d.shape == (5, 2, 1, 8, 4) and d.dtype == np.int16
        assert d.chunks[1:] == d.shape[1:]   # whole sample per chunk row
        assert d[3, 0].ravel()[5] == 5 and d[3, 1].ravel()[5] == -5
        assert f['radar_adc/stamp_header_ns'][2] == 10 * 10**9 + 2000
        assert list(f['radar_adc/stamp_recv_ns'][:]) == list(range(100, 105))
        assert f['radar_adc'].attrs['frame_index_gap'] == 1
        assert f['radar_adc'].attrs['n_samples'] == 5
        assert f['adc_meta/frame_index'].dtype == np.uint32
        assert f['adc_meta/record_valid'].shape == (6, 4)
        assert f['adc_meta/prev_tail_sat'].shape == (6, 2)
        assert f['adc_meta'].attrs['frame_index_recorded_gaps'] == 0
        assert list(f['pc/row_count'][:]) == [1, 3]       # NaN row dropped
        assert f['pc/data'].shape == (4, 4)
        assert f['pc/stamp_recv_ns'].shape == (2,)


def test_frame_index_gaps_and_wrap(tmp_path):
    cap = CaptureFile(tmp_path / 'g.h5')
    cap.add_stream('m')
    c = cv.get_converter('raw_radar_msgs/msg/ADCChirpMeta')
    for k, idx in enumerate([0xFFFFFFFE, 0xFFFFFFFF, 1, 2]):  # wrap, skips 0
        put(cap, 'm', meta(idx, k), c, k)
    s = cap._streams['m']
    assert s.fi_gaps == 1 and s.fi_nonmono == 0
    cap.close()


def test_shape_mismatch_rejected_not_written(tmp_path):
    cap = CaptureFile(tmp_path / 's.h5')
    cap.add_stream('a')
    c = cv.get_converter('raw_radar_msgs/msg/ADCDataCube')
    put(cap, 'a', adc(0), c, 1)
    with pytest.raises(ValueError):
        put(cap, 'a', adc(1, dims=(1, 9, 4)), c, 2)
    assert cap.count('a') == 1
    cap.close()
    with h5py.File(tmp_path / 's.h5', 'r') as f:
        assert f['a/data'].shape[0] == 1
        assert f['a/stamp_recv_ns'].shape[0] == 1
        assert f['a'].attrs['n_errors'] == 1


def test_flush_makes_open_file_readable_midway(tmp_path):
    cap = CaptureFile(tmp_path / 'f.h5', flush_period_s=0.0)
    cap.add_stream('a')
    c = cv.get_converter('raw_radar_msgs/msg/ADCDataCube')
    put(cap, 'a', adc(0), c, 1)
    with h5py.File(tmp_path / 'f.h5', 'r', swmr=False) as f:   # same process
        pass
    cap.close()


def test_small_messages():
    q = NS(x=0., y=0., z=0., w=1.)
    v = NS(x=1., y=2., z=3.)
    imu = NS(header=hdr(1, 2), orientation=q, angular_velocity=v,
             linear_acceleration=v)
    assert cv.get_converter('sensor_msgs/Imu').fn(imu, {}).fields['data'].shape == (10,)
    od = NS(header=hdr(1, 2), pose=NS(pose=NS(position=v, orientation=q)),
            twist=NS(twist=NS(linear=v, angular=v)))
    s = cv.get_converter('nav_msgs/Odometry').fn(od, {})
    assert s.header_ns == 10**9 + 2 and s.fields['data'].shape == (13,)
    tw = cv.get_converter('geometry_msgs/Twist').fn(NS(linear=v, angular=v), {})
    assert tw.header_ns == -1 and tw.fields['data'][3] == 1.0
    img = NS(header=hdr(0, 5), encoding='bgr8', height=2, width=3, step=10,
             data=bytes(range(20)))
    im = cv.get_converter('sensor_msgs/Image').fn(img, {}).fields['data']
    assert im.shape == (2, 3, 3) and im[1, 0, 0] == 10      # step padding skipped
    with pytest.raises(KeyError):
        cv.get_converter('foo/Bar')


def test_parse_streams():
    got = parse_streams({'a.topic': 't', 'a.type': 'x/Y', 'a.qos': 'reliable',
                         'b.topic': 'u', 'b.type': 'x/Z'})
    assert set(got) == {'a', 'b'} and got['a']['qos'] == 'reliable'
    with pytest.raises(ValueError):
        parse_streams({'a.topic': 't'})
    with pytest.raises(ValueError):
        parse_streams({'a.topic': 't', 'a.type': 'x', 'a.bogus': 1})


def test_legacy_export(tmp_path):
    path = tmp_path / 'l.h5'
    cap = CaptureFile(path)
    cap.add_stream('radar_adc', {'legacy_folder': 'radar_0_adc',
                                  'columns': ['real', 'imag']})   # columns: set by describe() in the node)
    cap.add_stream('radar_pc')
    cc = cv.get_converter('raw_radar_msgs/msg/ADCDataCube')
    pc = cv.get_converter('sensor_msgs/msg/PointCloud2')
    for i in range(3):
        put(cap, 'radar_adc', adc(i), cc, 100 * (i + 1))
        put(cap, 'radar_pc', cloud([[i, 0, 0, 0]]), pc, 100 * (i + 1) + 5)
    cap.close()
    counts = export(str(path), str(tmp_path / 'out'), reference='radar_adc')
    assert counts['radar_adc'] == 3 and counts['radar_pc'] == 2
    a = np.load(tmp_path / 'out' / 'radar_0_adc' / 'frame_1.npy')
    assert a.dtype == np.complex128 and a.shape == (1, 8, 4)
    assert os.path.exists(tmp_path / 'out' / 'radar_pc' / 'frame_2.npy')

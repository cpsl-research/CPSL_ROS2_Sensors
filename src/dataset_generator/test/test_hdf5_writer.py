"""Writer/converter checks with fake messages (no ROS graph, no hardware).

These test the code, they are not a capture result.
"""
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

h5py = pytest.importorskip('h5py')

from dataset_generator import converters as cv  # noqa: E402
from dataset_generator.stream_config import (  # noqa: E402
    parse_streams, parse_tf_track, tf_stream_name)
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
        assert f.attrs['closed_cleanly'] and f.attrs['format_version'] == 4
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


def test_close_survives_second_sigint(tmp_path, monkeypatch):
    import signal
    cap = CaptureFile(tmp_path / 'c.h5')
    cap.add_stream('s')
    orig = cap._write_stream_attrs

    def interrupted():
        signal.raise_signal(signal.SIGINT)   # a second ^C mid-close
        orig()
    monkeypatch.setattr(cap, '_write_stream_attrs', interrupted)
    before = signal.getsignal(signal.SIGINT)
    cap.close()
    assert signal.getsignal(signal.SIGINT) is before
    with h5py.File(tmp_path / 'c.h5', 'r') as f:
        assert bool(f.attrs['closed_cleanly']) is True


def _launch_path_params(param_file, ns=''):
    """Load a shipped YAML the way record_dataset.launch.py does."""
    import yaml
    from dataset_generator.launch_params import write_params
    cfg = os.path.join(os.path.dirname(__file__), '..', 'configs', param_file)
    with open(write_params(cfg, ns)) as f:
        return yaml.safe_load(f)


def _configs():
    d = os.path.join(os.path.dirname(__file__), '..', 'configs')
    return sorted(x for x in os.listdir(d) if x.endswith('.yaml'))


def _lists(node, key=''):
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _lists(v, k)
    elif isinstance(node, list):
        yield key, node


@pytest.mark.parametrize('name', _configs())
def test_shipped_config_loads_through_launch_path(name):
    doc = _launch_path_params(name)
    assert 'ros__parameters' in doc['dataset_generator']
    for k, v in _lists(doc):
        # rclpy rejects mixed-type sequences ("Sequence should be of same type")
        assert len({type(x) for x in v}) <= 1, k
        if k == 'fields':
            assert all(isinstance(x, str) for x in v), (name, v)


def test_radar_sar_field_names_stay_strings():
    doc = _launch_path_params('radar_sar.yaml')
    fields = dict(_lists(doc))['fields']
    assert fields == ['x', 'y', 'z', 'vel']


@pytest.mark.parametrize('name', _configs())
def test_shipped_config_parses_in_rcl(name):
    """The real check: rcl's own parser must accept the generated file."""
    rclpy = pytest.importorskip('rclpy')
    from dataset_generator.launch_params import write_params
    cfg = os.path.join(os.path.dirname(__file__), '..', 'configs', name)
    path = write_params(cfg)
    rclpy.init(args=['--ros-args', '--params-file', path])
    try:
        node = rclpy.create_node('dataset_generator')
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_rover_pro_radar_streams_and_mount_in_config_streams(tmp_path):
    doc = _launch_path_params('rover_pro_radar.yaml')
    raw = {}
    for n, c in doc['dataset_generator']['ros__parameters']['streams'].items():
        for k, v in c.items():          # flatten like get_parameters_by_prefix
            raw['%s.%s' % (n, k)] = v
    streams = parse_streams(raw)
    got = {n: (c['type'], c['qos']) for n, c in streams.items()}
    assert got == {
        'radar_adc': ('raw_radar_msgs/msg/ADCDataCube', 'reliable'),
        'adc_meta': ('raw_radar_msgs/msg/ADCChirpMeta', 'reliable'),
        'odom': ('nav_msgs/msg/Odometry', 'reliable'),
        'cmd_vel': ('geometry_msgs/msg/Twist', 'reliable')}
    assert streams['adc_meta']['meta_for'] == 'radar_adc'
    # same dict shape the recorder stores as the root attr
    cs = {n: {k: (list(v) if isinstance(v, (list, tuple)) else v)
              for k, v in c.items()} for n, c in streams.items()}
    path = tmp_path / 'r.h5'
    cap = CaptureFile(path, {'config_streams': cs}, flush_period_s=0.0)
    cap.add_stream('radar_adc', {'topic': 't'}, keep_stamps=True)
    cap.close()
    import json
    with h5py.File(path, 'r') as f:
        back = f.attrs['config_streams']
        back = json.loads(back) if isinstance(back, (str, bytes)) else back
    m = back['radar_adc']
    assert m['mount_frame'] == 'base_link'
    assert list(m['mount_xyz']) == [-0.05, -0.02, 0.0]
    assert list(m['mount_rpy']) == [0.0, 0.0, -1.5708]
    assert 'right-looking' in m['mount_note']


def test_radar_cfg_attrs_roundtrip(tmp_path):
    import json
    p = str(tmp_path / 'c.h5')
    cap = CaptureFile(p, {'host': 'h'})
    cap.add_stream('radar_adc', {'topic': 't'})
    cap.update_stream_attrs('radar_adc', {
        'radar_cfg_path': '/x/a.cfg', 'radar_cfg_text': 'frameCfg 0 0 1\n',
        'radar_cfg_sha256': 'ab', 'radar_cfg_status': 'ok',
        'radar_cfg_changed': False,
        'radar_cfg_metrics': json.dumps({'n_rx': 4}, sort_keys=True)})
    cap.close()
    with h5py.File(p) as f:
        a = f['radar_adc'].attrs
        assert f.attrs['format_version'] == 4
        assert a['radar_cfg_text'] == 'frameCfg 0 0 1\n'
        assert a['radar_cfg_status'] == 'ok' and not a['radar_cfg_changed']
        assert json.loads(a['radar_cfg_metrics'])['n_rx'] == 4


def test_recorder_store_cfg(tmp_path):
    """The recorder's writer-thread cfg handling, on a stub (no ROS graph)."""
    pytest.importorskip('rclpy')
    pytest.importorskip('std_msgs')
    import json
    from dataset_generator.dataset_recorder import DatasetRecorder
    cfg = tmp_path / 'r.cfg'
    cfg.write_text('% c\nchannelCfg 15 1 0\nadcCfg 2 1\n'
                   'profileCfg 0 77 7 6 57 0 0 70 1 256 5209 0 0 30\n'
                   'chirpCfg 0 0 0 0 0 0 0 1\n'
                   'frameCfg 0 0 128 0 100 1 0\nlvdsStreamCfg -1 0 1 0\n')
    p = str(tmp_path / 'c.h5')
    cap = CaptureFile(p)
    cap.add_stream('radar_adc', {})
    log = NS(warning=lambda *a, **k: None, error=lambda *a, **k: None,
             info=lambda *a, **k: None)
    me = NS(cap=cap, _cfg_writer_path={}, get_logger=lambda: log)
    DatasetRecorder._store_cfg(me, 'radar_adc', str(cfg))
    DatasetRecorder._store_cfg(me, 'radar_adc', str(cfg))          # repeat
    DatasetRecorder._store_cfg(me, 'radar_adc', '/other/path.cfg')  # change
    cap.close()
    with h5py.File(p) as f:
        a = f['radar_adc'].attrs
        assert a['radar_cfg_status'] == 'ok'
        assert a['radar_cfg_path'] == str(cfg)
        assert a['radar_cfg_text'] == cfg.read_text()
        assert a['radar_cfg_changed']
        assert len(a['radar_cfg_sha256']) == 64
        assert json.loads(a['radar_cfg_metrics'])['n_rx'] == 4
    # unreadable path: stored with a status, no text
    cap = CaptureFile(str(tmp_path / 'd.h5'))
    cap.add_stream('radar_adc', {})
    me = NS(cap=cap, _cfg_writer_path={}, get_logger=lambda: log)
    DatasetRecorder._store_cfg(me, 'radar_adc', '/nonexistent/x.cfg')
    cap.close()
    with h5py.File(str(tmp_path / 'd.h5')) as f:
        a = f['radar_adc'].attrs
        assert a['radar_cfg_status'].startswith('unreadable')
        assert a['radar_cfg_path'] == '/nonexistent/x.cfg'
        assert 'radar_cfg_text' not in a


# ---- transforms (format v4) -------------------------------------------------
def tfs(parent, child, sec, x=0.0, frame_slash=False):
    t = NS(x=x, y=2.0, z=3.0)
    q = NS(x=0.0, y=0.0, z=0.0, w=1.0)
    p = ('/' if frame_slash else '') + parent
    return NS(header=hdr(sec, 5, p), child_frame_id=child,
              transform=NS(translation=t, rotation=q))


def test_parse_tf_track():
    assert parse_tf_track(['odom->base_link']) == [('odom', 'base_link')]
    assert parse_tf_track(['/odom -> /base_link', 'odom->base_link',
                           'map->odom']) == [('odom', 'base_link'),
                                             ('map', 'odom')]
    for off in (None, '', [''], [], ['', ' ']):
        assert parse_tf_track(off) == []
    for bad in ('odom', 'a->b->c', '->b', 'a->', 'a=>b', 'a-b'):
        with pytest.raises(ValueError):
            parse_tf_track([bad])
    assert tf_stream_name('a/b', 'c') == 'tf/a.b__c'


def test_tf_converter_filters_to_tracked_pairs():
    msg = NS(transforms=[tfs('odom', 'base_link', 3, x=1.5),
                         tfs('odom', 'other', 3),
                         tfs('base_link', 'radar', 3, frame_slash=True)])
    rows = cv.tf_transforms(msg, [('odom', 'base_link')])
    assert [(r.parent, r.child) for r in rows] == [('odom', 'base_link')]
    assert rows[0].stamp_ns == 3 * 10**9 + 5
    assert list(rows[0].data) == [1.5, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
    # no pair list = everything, with the leading '/' stripped
    allrows = cv.tf_transforms(msg)
    assert len(allrows) == 3 and allrows[2].parent == 'base_link'
    assert cv.tf_transforms(msg, []) == []
    s = cv.get_converter('tf2_msgs/TFMessage').fn(
        msg, {'pairs': [('odom', 'other')]})
    assert s.fields['data'].shape == (1, 7) and s.ragged == ('data',)


def test_tf_pair_streams_and_static_roundtrip(tmp_path):
    p = tmp_path / 't.h5'
    cap = CaptureFile(p, {'tf_track': ['odom->base_link', 'a/b->c']},
                      flush_period_s=0.0)
    n1 = tf_stream_name('odom', 'base_link')
    n2 = tf_stream_name('a/b', 'c')
    assert (n1, n2) == ('tf/odom__base_link', 'tf/a.b__c')
    for n, par, ch in ((n1, 'odom', 'base_link'), (n2, 'a/b', 'c')):
        cap.add_stream(n, {'parent_frame': par, 'child_frame': ch,
                           'columns': cv.TF_COLUMNS})
    for bad in ('tf/', 'tf/a/b', 'x.y', 'a/b'):
        with pytest.raises(ValueError):
            cap.add_stream(bad)
    rows = cv.tf_transforms(NS(transforms=[tfs('odom', 'base_link', 4)]))
    for i in range(3):
        cap.append(n1, rows[0].stamp_ns + i, 100 + i, {'data': rows[0].data})
    cap.write_tf_static([('base_link', 'radar', [1, 2, 3], [0, 0, 0, 1],
                          7, 8)])
    cap.write_tf_static([('base_link', 'radar', [1, 2, 3], [0, 0, 0, 1],
                          7, 8),
                         ('base_link', 'lidar', [0, 0, 1], [0, 0, 1, 0],
                          9, 10)])      # a new pair rewrites the group
    cap.close()
    with h5py.File(p, 'r') as f:
        assert f.attrs['format_version'] == 4
        g = f['tf/odom__base_link']
        assert g['data'].shape == (3, 7)
        assert g.attrs['parent_frame'] == 'odom'
        assert g.attrs['child_frame'] == 'base_link'
        assert g.attrs['n_samples'] == 3
        assert g['stamp_header_ns'][1] == 4 * 10**9 + 5 + 1
        assert f['tf/a.b__c'].attrs['n_samples'] == 0     # silent pair
        assert 'data' not in f['tf/a.b__c']
        st = f['tf_static']
        assert list(st['parent'].asstr()[:]) == ['base_link', 'base_link']
        assert list(st['child'].asstr()[:]) == ['radar', 'lidar']
        assert st['xyz'].shape == (2, 3) and st['quat_xyzw'].shape == (2, 4)
        assert list(st['quat_xyzw'][1]) == [0, 0, 1, 0]
        assert list(st['stamp_ns'][:]) == [7, 9]
        assert list(st['recv_ns'][:]) == [8, 10]
    # an empty tf_static is still a valid (N=0) group
    cap = CaptureFile(tmp_path / 'e.h5')
    cap.write_tf_static([])
    cap.close()
    with h5py.File(tmp_path / 'e.h5', 'r') as f:
        assert f['tf_static/xyz'].shape == (0, 3)
        assert f['tf_static/parent'].shape == (0,)


def test_recorder_tf_static_dedup_and_v4_attrs(tmp_path):
    """The recorder's /tf_static handling on a stub (no ROS graph)."""
    pytest.importorskip('rclpy')
    pytest.importorskip('std_msgs')
    import json
    from dataset_generator.dataset_recorder import DatasetRecorder
    calls = []

    def _write_tf_static():
        pass

    me = NS(_tfs={}, _gate_open=True, _write_tf_static=_write_tf_static,
            _queue_call=lambda fn: calls.append(fn.__name__))
    m1 = NS(transforms=[tfs('base_link', 'radar', 1, x=1.0)])
    DatasetRecorder._on_tf_static(me, m1)
    first = dict(me._tfs)
    DatasetRecorder._on_tf_static(me, m1)                # identical repeat
    assert calls == ['_write_tf_static'] and me._tfs == first
    DatasetRecorder._on_tf_static(
        me, NS(transforms=[tfs('base_link', 'radar', 1, x=2.0)]))   # changed
    DatasetRecorder._on_tf_static(
        me, NS(transforms=[tfs('base_link', 'lidar', 1)]))          # new pair
    assert len(calls) == 3 and len(me._tfs) == 2
    # gate not open yet: stored, nothing queued
    me2 = NS(_tfs={}, _gate_open=False, _write_tf_static=_write_tf_static,
             _queue_call=lambda fn: calls.append('late'))
    DatasetRecorder._on_tf_static(me2, m1)
    assert len(me2._tfs) == 1 and 'late' not in calls
    # the v4 root attrs, as the recorder passes them at gate open
    path = tmp_path / 'v4.h5'
    cap = CaptureFile(path, {
        'start_gate': {'timeout_s': 30.0, 'allow_missing': True,
                       'wait_s': 1.5, 'opened_recv_ns': 123},
        'streams_missing': ['odom'], 'tf_track': ['odom->base_link'],
        'tf_topic': '/tf', 'tf_static_topic': '/tf_static'})
    cap.add_stream('odom')
    me3 = NS(cap=cap, _tfs=me._tfs)
    DatasetRecorder._write_tf_static(me3)
    cap.close()
    with h5py.File(path, 'r') as f:
        a = f.attrs
        assert a['format_version'] == 4
        gate = json.loads(a['start_gate'])
        assert gate['allow_missing'] is True and gate['wait_s'] == 1.5
        assert list(a['streams_missing']) == ['odom']
        assert list(a['tf_track']) == ['odom->base_link']
        assert a['tf_topic'] == '/tf' and a['tf_static_topic'] == '/tf_static'
        assert f['tf_static/xyz'].shape == (2, 3)
        assert f['tf_static/xyz'][0][0] == 2.0       # the changed value won


def test_legacy_export_skips_tf_groups(tmp_path):
    path = tmp_path / 'v4l.h5'
    cap = CaptureFile(path)
    cap.add_stream('radar_pc')
    cap.add_stream('tf/odom__base_link')
    pc = cv.get_converter('sensor_msgs/msg/PointCloud2')
    row = cv.tf_transforms(NS(transforms=[tfs('odom', 'base_link', 1)]))[0]
    for i in range(2):
        put(cap, 'radar_pc', cloud([[i, 0, 0, 0]]), pc, 100 * (i + 1))
        cap.append('tf/odom__base_link', row.stamp_ns, 100 * (i + 1) - 1,
                   {'data': row.data})
    cap.write_tf_static([('base_link', 'radar', [0, 0, 0], [0, 0, 0, 1], 1, 2)])
    cap.close()
    counts = export(str(path), str(tmp_path / 'out'))
    assert counts == {'radar_pc': 2}
    assert sorted(os.listdir(tmp_path / 'out')) == ['radar_pc']


@pytest.mark.parametrize('name', ['ugv_dataset.yaml', 'radar_sar.yaml',
                                  'rover_pro_radar.yaml'])
def test_v2_presets_have_gate_and_tf_params(name):
    p = _launch_path_params(name)['dataset_generator']['ros__parameters']
    assert p['start_timeout_s'] >= 0 and p['allow_missing'] is False
    pairs = parse_tf_track(p['tf_track'])
    assert pairs == ([] if name == 'radar_sar.yaml'
                     else [('odom', 'base_link')])

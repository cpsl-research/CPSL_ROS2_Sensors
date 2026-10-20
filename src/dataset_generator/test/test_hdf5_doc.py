"""Doc-sync checks: docs/hdf5_format.md and docs/reading_datasets.md against the real writer.

A fixture file is written with the real CaptureFile and converters from fake
messages (no ROS, no hardware; this tests the docs, it is not a capture
result). The column/dtype/shape tables are parsed out of the markdown and
asserted against that file, every python block of the reader guide is run
against it, and hdf5_to_legacy is checked against the documented mapping.
Attributes the recorder node (not the writer) adds, such as topic, qos and
resolved_topic, are not in a writer-only fixture and are not checked here.
"""
import re
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

h5py = pytest.importorskip('h5py')

from dataset_generator import converters as cv  # noqa: E402
from dataset_generator.hdf5_to_legacy import export  # noqa: E402
from dataset_generator.hdf5_writer import CaptureFile  # noqa: E402

DOCS = Path(__file__).resolve().parent.parent / 'docs'
FORMAT_MD = (DOCS / 'hdf5_format.md').read_text()
READING_MD = (DOCS / 'reading_datasets.md').read_text()

# attrs the recorder node adds; a writer-only fixture does not have them
NODE_ATTRS = {'topic', 'resolved_topic', 'qos', 'qos_depth', 'mode', 'legacy_folder',
              'n_dropped_node'}
DIMS = (4, 8, 16)   # rx, sample, chirp
N_CHIRPS = DIMS[2]


# ---------------------------------------------------------------- fixture
def _v(x, y, z):
    return NS(x=x, y=y, z=z)


def _q(z, w):
    return NS(x=0., y=0., z=z, w=w)


def _hdr(t, frame):
    s = int(t)
    return NS(stamp=NS(sec=s, nanosec=int(round((t - s) * 1e9))), frame_id=frame)


def _adc(t):
    n = int(np.prod(DIMS))
    return NS(header=_hdr(t, 'radar'),
              layout=NS(dim=[NS(label=l, size=s, stride=0)
                             for l, s in zip(('rx', 'sample', 'chirp'), DIMS)]),
              real_data=list(range(n)), imag_data=[-x for x in range(n)])


def _meta(t, i):
    z = [0] * N_CHIRPS
    return NS(header=_hdr(t, 'radar'), frame_index=i,
              record_valid=[True] * N_CHIRPS, adc_complete=[True] * N_CHIRPS,
              flags=z, global_chirp_idx=z, run_idx=z, ts_ticks=z, sat_slices=z,
              sat_ref_lag=z, sat_slices_aligned=z, has_time=[True] * N_CHIRPS,
              time_interpolated=[False] * N_CHIRPS, t_s=[0.1] * N_CHIRPS,
              prev_tail_sat=[-1, -1], run_known=True, run_idx_frame=3,
              records_valid=N_CHIRPS, records_invalid=0, records_other_run=0)


def _cloud(t):
    arr = np.asarray([[1, 0, 0, 5], [2, 1, 0, -1], [3, -1, 0.5, 0]], np.float32)
    fields = [NS(name=n, offset=4 * k, datatype=7, count=1)
              for k, n in enumerate(('x', 'y', 'z', 'vel'))]
    return NS(header=_hdr(t, 'radar'), fields=fields, width=len(arr), height=1,
              point_step=16, is_bigendian=False, data=arr.tobytes())


def _write_fixture(path):
    cap = CaptureFile(path, {'capture_mode': 'all'}, flush_period_s=0)

    def stream(name, typ, msgs, opts=None, legacy=None):
        c = cv.get_converter(typ)
        opts = opts or {}
        attrs = {'msg_type': typ, **c.describe(msgs[0][1], opts)}
        if legacy:
            attrs['legacy_folder'] = legacy
        cap.add_stream(name, attrs)
        for t, m in msgs:
            s = c.fn(m, opts)
            cap.append(name, s.header_ns, int((t + .002) * 1e9), s.fields, s.ragged)

    r5 = range(5)
    stream('radar_adc', 'raw_radar_msgs/msg/ADCDataCube',
           [(10 + .1 * i, _adc(10 + .1 * i)) for i in r5], legacy='radar_0_adc')
    stream('adc_meta', 'raw_radar_msgs/msg/ADCChirpMeta',
           [(10 + .1 * i, _meta(10 + .1 * i, i)) for i in r5])
    stream('radar_pc', 'sensor_msgs/msg/PointCloud2',
           [(10.05 + .1 * i, _cloud(10.05 + .1 * i)) for i in r5],
           {'fields': ['x', 'y', 'z', 'vel']}, legacy='radar_0_pc')
    stream('camera', 'sensor_msgs/msg/Image',
           [(10.03 + .1 * i, NS(header=_hdr(10.03 + .1 * i, 'cam'), encoding='rgb8',
                                height=2, width=3, step=9, data=bytes(range(18))))
            for i in r5])
    stream('imu', 'sensor_msgs/msg/Imu',
           [(10 + .025 * i, NS(header=_hdr(10 + .025 * i, 'imu'), orientation=_q(0, 1),
                               angular_velocity=_v(0, 0, .1 * i),
                               linear_acceleration=_v(0, 0, 9.8)))
            for i in range(20)], legacy='imu_data')

    def odom(t):
        return NS(header=_hdr(t, 'odom'), child_frame_id='base_link',
                  pose=NS(pose=NS(position=_v(.5 * (t - 10), 0, 0), orientation=_q(0, 1))),
                  twist=NS(twist=NS(linear=_v(.5, 0, 0), angular=_v(0, 0, 0))))
    stream('odom', 'nav_msgs/msg/Odometry',
           [(10 + .1 * i, odom(10 + .1 * i)) for i in range(10)], legacy='vehicle_odom')
    twist = NS(linear=_v(.5, 0, 0), angular=_v(0, 0, 0))
    stream('cmd_vel', 'geometry_msgs/msg/Twist', [(10 + .2 * i, twist) for i in r5])
    stream('twist_st', 'geometry_msgs/msg/TwistStamped',
           [(10 + .2 * i, NS(header=_hdr(10 + .2 * i, 'odom'), twist=twist)) for i in r5])
    stream('xform', 'geometry_msgs/msg/TransformStamped',
           [(10 + .2 * i, NS(header=_hdr(10 + .2 * i, 'odom'), child_frame_id='base_link',
                             transform=NS(translation=_v(1, 2, 3), rotation=_q(0, 1))))
            for i in r5])
    stream('vec3', 'geometry_msgs/msg/Vector3Stamped',
           [(10 + .2 * i, NS(header=_hdr(10 + .2 * i, 'imu'), vector=_v(1, 2, 3)))
            for i in r5])
    stream('cam_info', 'sensor_msgs/msg/CameraInfo',
           [(10.0, NS(header=_hdr(10.0, 'cam'), width=4, height=3, k=[1.] * 9,
                      r=[0.] * 9, p=[0.] * 12, d=[.1] * 5,
                      distortion_model='plumb_bob'))])

    def tfm(t):
        return NS(transforms=[NS(header=_hdr(t, 'odom'), child_frame_id='base_link',
                                 transform=NS(translation=_v(1, 2, 3), rotation=_q(0, 1)))
                              for _ in range(2)])
    stream('tf_msg', 'tf2_msgs/msg/TFMessage', [(10 + .2 * i, tfm(10 + .2 * i)) for i in r5])

    cap.add_stream('tf/odom__base_link', {'columns': cv.TF_COLUMNS,
                                          'parent_frame': 'odom', 'child_frame': 'base_link'})
    for i in range(10):
        t = 10 + .1 * i
        row = cv.tf_transforms(NS(transforms=[NS(
            header=_hdr(t, 'odom'), child_frame_id='base_link',
            transform=NS(translation=_v(.5 * (t - 10), 0, 0), rotation=_q(0, 1)))]))[0]
        cap.append('tf/odom__base_link', row.stamp_ns, int((t + .002) * 1e9),
                   {'data': row.data})
    s2 = 2 ** .5 / 2
    cap.write_tf_static([('base_footprint', 'base_link', [0, 0, .1], [0, 0, 0, 1], 0, 0),
                         ('base_link', 'radar', [.1, 0, .2], [0, 0, s2, s2], 0, 0)])
    cap.close()


@pytest.fixture(scope='module')
def h5file(tmp_path_factory):
    path = tmp_path_factory.mktemp('doc') / 'capture.h5'
    _write_fixture(path)
    return str(path)


# ---------------------------------------------------------------- doc parsing
def _section(md, heading):
    m = re.search(r'^## %s.*?$(.*?)(?=^## |\Z)' % re.escape(heading), md, re.S | re.M)
    assert m, 'section %r missing from the doc' % heading
    return m.group(1)


def _rows(text):
    """Markdown table rows (cells stripped), header and separator dropped."""
    out = []
    for line in text.splitlines():
        if line.startswith('|') and not re.match(r'^\|[\s:-]+\|', line):
            out.append([c.strip() for c in line.strip().strip('|').split('|')])
    return out[1:]


def _ticks(cell):
    return re.findall(r'`([^`]*)`', cell)


def _dtypes(cell):
    names = {'int16': 'int16', 'float32': 'float32', 'float64': 'float64',
             'u8': 'uint8', 'u16': 'uint16', 'f32': 'float32'}
    return {np.dtype(names[w]) for w in re.findall(r'\b(int16|float32|float64|u8|u16|f32)\b', cell)}


def _shapes(cell):
    return [[t.strip() for t in s.split(',')] for s in re.findall(r'\(([^()]*)\)', cell)]


def _shape_ok(tokens, shape, n, ragged_total):
    if len(tokens) != len(shape):
        return False
    for tok, dim in zip(tokens, shape):
        if tok.isdigit() and int(tok) != dim:
            return False
    if tokens[0] == 'N' and shape[0] != n:
        return False
    if tokens[0] == 'rows' and shape[0] != ragged_total:
        return False
    return True


def _expand(cell):
    cell = re.sub(r'\([^)]*\)', '', cell).replace('`', '')
    out = []
    for tok in cell.split():
        m = re.fullmatch(r'([A-Za-z])(\d+)\.\.\1(\d+)', tok)
        if m:
            out += ['%s%d' % (m.group(1), i) for i in range(int(m.group(2)), int(m.group(3)) + 1)]
        else:
            out.append(tok)
    return out


TYPE_STREAM = {
    'ADCDataCube': 'radar_adc', 'PointCloud2': 'radar_pc', 'Image': 'camera', 'Imu': 'imu',
    'Odometry': 'odom', 'Twist': 'cmd_vel', 'TwistStamped': 'twist_st',
    'TransformStamped': 'xform', 'Vector3Stamped': 'vec3', 'CameraInfo': 'cam_info',
}
COLUMN_TYPES = {'Imu', 'Odometry', 'Twist', 'TwistStamped', 'TransformStamped',
                'Vector3Stamped', 'CameraInfo', 'TFMessage'}


def _attr_list(v):
    return [x.decode() if isinstance(x, bytes) else str(x) for x in v]


def test_columns_dtypes_table(h5file):
    rows = _rows(_section(FORMAT_MD, 'Columns and dtypes per type').split('ADCChirpMeta datasets')[0])
    names = {r[0].split(' ')[0].strip('`') for r in rows}
    assert names == set(TYPE_STREAM) | {'ADCChirpMeta', 'TFMessage'}, names
    with h5py.File(h5file, 'r') as f:
        for typ, dcell, ccell, acell in rows:
            typ = typ.split(' ')[0]
            if typ == 'ADCChirpMeta':
                assert 'none' in dcell and ccell == 'none' and 'no `frame_id`' in acell
                a = f['adc_meta'].attrs
                assert 'data' not in f['adc_meta'] and 'columns' not in a and 'frame_id' not in a
                continue
            g = f[TYPE_STREAM.get(typ, 'tf_msg')]
            n = g.attrs['n_samples']
            ragged = 'ragged' in dcell
            assert ('row_count' in g) == ragged, typ
            d = g['data']
            assert d.dtype in _dtypes(dcell), (typ, d.dtype, dcell)
            total = int(g['row_count'][:].sum()) if ragged else None
            assert any(_shape_ok(s, d.shape, n, total) for s in _shapes(dcell)), \
                (typ, d.shape, dcell)
            if '`-1`' in dcell:
                assert (g['stamp_header_ns'][:] == -1).all(), typ
            if typ in COLUMN_TYPES:
                assert _attr_list(g.attrs['columns']) == _expand(ccell), typ
                assert d.shape[1] == len(_expand(ccell)), typ
            if typ == 'ADCDataCube':
                assert _attr_list(g.attrs['columns']) == [t for t in _ticks(ccell)
                                                          if t in ('real', 'imag')]
            if typ == 'PointCloud2':
                assert _attr_list(g.attrs['columns']) == ['x', 'y', 'z', 'vel']
            # "Other attrs": every named attr the writer sets must exist; 'only' / 'none' forms
            wanted = {t for t in _ticks(acell) if re.fullmatch(r'[a-z_]+', t)} - NODE_ATTRS
            wanted = {w for w in wanted if not w.startswith('radar_cfg')}
            if 'only' in acell:
                assert 'frame_id' not in g.attrs, typ
            for w in wanted:
                assert w in g.attrs, (typ, w)
            if typ == 'ADCDataCube':
                m = re.search(r'`axes`=`\[([^\]]*)\]`', acell)
                assert _attr_list(g.attrs['axes']) == [s.strip() for s in m.group(1).split(',')]
            if typ == 'Odometry':
                assert 'child_frame_id' in g.attrs


def test_chirp_meta_table(h5file):
    sec = _section(FORMAT_MD, 'Columns and dtypes per type').split('ADCChirpMeta datasets')[1]
    rows = _rows(sec.split('Lost-frame')[0])
    seen = set()
    with h5py.File(h5file, 'r') as f:
        g = f['adc_meta']
        for names, dt, shape in rows:
            toks = [t.strip() for t in shape.strip('()').split(',') if t.strip()]
            toks = [str(N_CHIRPS) if t == 'C' else t for t in toks]
            for name in (x.strip() for x in names.split(',')):
                seen.add(name)
                d = g[name]
                assert d.dtype == np.dtype(dt), (name, d.dtype, dt)
                assert _shape_ok(toks, d.shape, g.attrs['n_samples'], None), \
                    (name, d.shape, shape)
        extra = set(g) - seen - {'stamp_header_ns', 'stamp_recv_ns'}
        assert not extra, 'datasets missing from the doc table: %s' % extra


def test_format_version_and_root_attrs(h5file):
    with h5py.File(h5file, 'r') as f:
        const = int(re.search(r'writer constant is (\d+)', FORMAT_MD).group(1))
        assert f.attrs['format_version'] == const
        assert f.attrs['format'] == re.search(r'`format="([^"]+)"`', FORMAT_MD).group(1)
        for k in ('closed_cleanly', 'end_recv_ns', 'last_flush_recv_ns'):
            assert k in f.attrs and ('`%s`' % k) in FORMAT_MD
        assert f.attrs['closed_cleanly']


def test_tf_storage_documented(h5file):
    sec = _section(FORMAT_MD, 'Transforms')
    stat = re.search(r'\*\*`/tf_static`\*\*:(.*?)\n\n', sec, re.S).group(1)
    pair = re.search(r'\*\*Tracked `/tf` pairs\*\*:(.*?)\n\n', sec, re.S).group(1)
    with h5py.File(h5file, 'r') as f:
        g = f['tf_static']
        for name in ('parent', 'child', 'xyz', 'quat_xyzw', 'stamp_ns', 'recv_ns'):
            assert '`%s`' % name in stat and name in g, name
        assert '`n_transforms`' in stat and g.attrs['n_transforms'] == 2
        for name, width in re.findall(r'`(xyz|quat_xyzw)` \(N,(\d)\)', stat):
            assert g[name].shape == (2, int(width)) and g[name].dtype == np.float64
        assert g['stamp_ns'].dtype == np.int64 and g['recv_ns'].dtype == np.int64
        assert g['parent'].asstr()[0] == 'base_footprint'
        p = f['tf/odom__base_link']
        assert 'group `/tf/<parent>__<child>`' in pair
        assert re.search(r'`data` float64 \(N,7\) `x y z qx qy qz qw`', pair)
        assert p['data'].shape == (10, 7) and p['data'].dtype == np.float64
        assert _attr_list(p.attrs['columns']) == 'x y z qx qy qz qw'.split()
        for k in ('columns', 'parent_frame', 'child_frame', 'n_samples', 'n_errors'):
            assert '`%s`' % k in pair and k in p.attrs, k
        assert 'no `frame_id`, no `row_count`' in pair
        assert 'frame_id' not in p.attrs and 'row_count' not in p


# ---------------------------------------------------------------- reader guide
def _blocks():
    return re.findall(r'```python\n(.*?)```', READING_MD, re.S)


def test_reader_blocks_run(h5file, capsys):
    blocks = _blocks()
    assert len(blocks) >= 6
    for k, b in enumerate(blocks):
        assert "'capture.h5'" in b
        exec(compile(b.replace("'capture.h5'", repr(h5file)), 'block%d' % k, 'exec'), {})
    assert 'radar_adc' in capsys.readouterr().out


def test_tf_example_result(h5file):
    block = next(b for b in _blocks() if 'static_chain' in b and 'v1_cloud' in b)
    ns = {}
    exec(compile(block.replace("'capture.h5'", repr(h5file)), 'tfblock', 'exec'), ns)
    # sensor point (1,0,0): radar->base_link = rot z 90 + (.1,0,.2) -> (.1,1,.2);
    # base_link->base_footprint adds z .1 -> (.1,1,.3); vel untouched
    np.testing.assert_allclose(ns['v1_cloud'][0], [0.1, 1.0, 0.3, 5.0], atol=1e-6)
    assert ns['v1_cloud'].dtype == np.float32 and ns['v1_cloud'].shape == (3, 4)
    # odom: cloud 2 is at t=10.25, odom x = 0.5 * 0.25 interpolated -> .1 + .125 in x, z .2
    np.testing.assert_allclose(ns['xyz_odom'][0], [0.225, 1.0, 0.2], atol=1e-6)
    # a chain with no static path raises as documented
    with pytest.raises(ValueError):
        with h5py.File(h5file, 'r') as f:
            ns['static_chain'](f, 'no_such_frame', 'base_footprint')


# ---------------------------------------------------------------- legacy export
def test_legacy_export_matches_mapping(h5file, tmp_path):
    table = _rows(_section(READING_MD, '5. Migrating').split('Recipe:')[0])
    v1 = {r[0]: r for r in table}
    adc_row = next(r for k, r in v1.items() if k.startswith('`<radar>_adc`'))
    pc_row = next(r for k, r in v1.items() if k.startswith('`<radar>_pc`'))
    assert 'complex128' in adc_row[0] and '(rx, sample, chirp)' in adc_row[0]
    assert 'float32' in pc_row[0] and '(n, 4)' in pc_row[0]

    with h5py.File(h5file, 'r') as f:
        names = sorted(n for n in f if n not in ('tf', 'tf_static')
                       and 'data' in f[n] and f[n]['stamp_recv_ns'].shape[0])
        # documented: the default reference is alphabetical, not YAML order
        first = names[0]
        n_first = f[first]['stamp_recv_ns'].shape[0]
        n_pc = f['radar_pc/stamp_recv_ns'].shape[0]
    out_default = tmp_path / 'default'
    counts = export(h5file, str(out_default))
    assert max(counts.values()) == counts[first] == n_first
    assert (out_default / 'cam_info').is_dir() and not (out_default / 'cam_info' / 'frame_1.npy').exists()

    out = tmp_path / 'ref'
    counts = export(h5file, str(out), reference='radar_pc')
    assert counts['radar_pc'] == n_pc
    # folder = legacy_folder attr, else the stream name; frames numbered from 0
    for folder in ('radar_0_adc', 'radar_0_pc', 'imu_data', 'vehicle_odom', 'camera'):
        assert (out / folder / 'frame_0.npy').exists() or (out / folder / 'frame_1.npy').exists()
    assert not (out / 'radar_adc').exists() and not (out / 'odom').exists()
    # no vehicle_vel folder and no PNG output
    assert not (out / 'vehicle_vel').exists() and not list(out.rglob('*.png'))

    adc = np.load(out / 'radar_0_adc' / 'frame_4.npy')
    assert adc.dtype == np.complex128 and adc.shape == DIMS      # [rx, sample, chirp]
    pc = np.load(out / 'radar_0_pc' / 'frame_0.npy')
    assert pc.dtype == np.float32 and pc.ndim == 2 and pc.shape[1] == 4
    imu = np.load(out / 'imu_data' / 'frame_0.npy')
    assert imu.shape == (10,) and imu.dtype == np.float64        # one latest row
    od = np.load(out / 'vehicle_odom' / 'frame_0.npy')
    assert od.shape == (13,)                                     # one latest row, no time column
    with h5py.File(h5file, 'r') as f:
        cols = _attr_list(f['odom'].attrs['columns'])
    assert cols[3:7] == ['qx', 'qy', 'qz', 'qw']                 # x-first quaternion, not v1's w-first
    img = np.load(out / 'camera' / 'frame_0.npy')
    assert img.dtype == np.uint8 and img.shape == (2, 3, 3)      # .npy, as published

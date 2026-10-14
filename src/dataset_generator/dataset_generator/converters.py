"""ROS message -> numpy converters for the HDF5 recorder.

No ROS imports: converters read attributes off whatever message object they
are handed, so tests can pass plain fake messages. Each converter returns a
``Sample`` (header stamp in ns, a dict of numpy fields, and the field names
that are ragged, i.e. vary in row count per sample).
"""
from collections import namedtuple

import numpy as np

Sample = namedtuple('Sample', 'header_ns fields ragged')
# per-type: fn(msg, opts) -> Sample ; describe(msg, opts) -> dict of stream attrs
Converter = namedtuple('Converter', 'fn describe default_qos')


def stamp_ns(msg):
    h = getattr(msg, 'header', None)
    if h is None:
        return -1
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def _fid(name):
    """Frame name as stored in attrs: no leading '/', like /tf_static."""
    return str(name).lstrip('/')


def _f64(*vals):
    return np.array(vals, dtype=np.float64)


# ---- ADCDataCube / ADCChirpMeta ---------------------------------------
def _adc_cube(msg, opts):
    dims = tuple(int(d.size) for d in msg.layout.dim)
    if not dims:
        raise ValueError('ADCDataCube with an empty layout')
    n = int(np.prod(dims))
    re = np.asarray(msg.real_data, dtype=np.int16)
    im = np.asarray(msg.imag_data, dtype=np.int16)
    if re.size != n or im.size != n:
        raise ValueError('ADCDataCube data size %d/%d != layout %s' %
                         (re.size, im.size, dims))
    data = np.stack([re.reshape(dims), im.reshape(dims)])
    return Sample(stamp_ns(msg), {'data': data}, ())


def _adc_cube_attrs(msg, opts):
    return {
        'columns': ['real', 'imag'],
        'axes': ['re_im'] + [d.label for d in msg.layout.dim],
        'frame_id': _fid(msg.header.frame_id),
    }


_META_ARRAYS = {
    'record_valid': np.bool_, 'adc_complete': np.bool_,
    'flags': np.uint16, 'global_chirp_idx': np.uint32,
    'run_idx': np.uint16, 'ts_ticks': np.uint64,
    'sat_slices': np.uint8, 'sat_ref_lag': np.uint8,
    'sat_slices_aligned': np.int16, 'has_time': np.bool_,
    'time_interpolated': np.bool_, 't_s': np.float64,
}
_META_SCALARS = {
    'frame_index': np.uint32, 'run_known': np.bool_,
    'run_idx_frame': np.uint16, 'records_valid': np.uint32,
    'records_invalid': np.uint32, 'records_other_run': np.uint32,
}


def _adc_meta(msg, opts):
    f = {}
    for k, dt in _META_ARRAYS.items():
        f[k] = np.asarray(getattr(msg, k), dtype=dt)
    f['prev_tail_sat'] = np.asarray(msg.prev_tail_sat, dtype=np.int16)
    for k, dt in _META_SCALARS.items():
        f[k] = np.asarray(getattr(msg, k), dtype=dt)
    return Sample(stamp_ns(msg), f, ())


# ---- PointCloud2 ---------------------------------------------------------
_PF = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def _cloud(msg, opts):
    want = list(opts.get('fields') or [f.name for f in msg.fields])
    by_name = {f.name: f for f in msg.fields}
    missing = [w for w in want if w not in by_name]
    if missing:
        raise ValueError('PointCloud2 has no field(s) %s' % missing)
    n = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw[:n * step].reshape(n, step)
    cols = []
    for w in want:
        f = by_name[w]
        dt = np.dtype(_PF[f.datatype])
        if getattr(msg, 'is_bigendian', False):
            dt = dt.newbyteorder('>')
        off = int(f.offset)
        col = np.ascontiguousarray(raw[:, off:off + dt.itemsize]).view(dt)
        cols.append(col.reshape(n))
    wide = (np.float64 if any(np.dtype(c.dtype).itemsize > 4 and
                              c.dtype.kind == 'f' for c in cols)
            else np.float32)
    pts = (np.stack(cols, axis=1).astype(wide) if cols
           else np.zeros((n, 0), wide))
    if opts.get('skip_nans', True) and pts.size:
        pts = pts[np.isfinite(pts).all(axis=1)]
    return Sample(stamp_ns(msg), {'data': pts}, ('data',))


def _cloud_attrs(msg, opts):
    return {'columns': list(opts.get('fields') or [f.name for f in msg.fields]),
            'frame_id': _fid(msg.header.frame_id),
            'skip_nans': bool(opts.get('skip_nans', True))}


# ---- Image -----------------------------------------------------------------
_ENC = {  # encoding -> (numpy dtype, channels)
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1), 'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3),
    '8UC3': (np.uint8, 3), 'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
}


def _image(msg, opts):
    if msg.encoding not in _ENC:
        raise ValueError('unsupported image encoding %r' % msg.encoding)
    dt, ch = _ENC[msg.encoding]
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    row = w * ch * np.dtype(dt).itemsize
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)[:h * step]
    img = raw.reshape(h, step)[:, :row].copy().view(dt).reshape(h, w, ch)
    if ch == 1:
        img = img[:, :, 0]
    return Sample(stamp_ns(msg), {'data': img}, ())


def _image_attrs(msg, opts):
    return {'encoding': msg.encoding, 'frame_id': _fid(msg.header.frame_id)}


# ---- small fixed-size messages ----------------------------------------------
def _imu(msg, opts):
    q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return Sample(stamp_ns(msg), {'data': _f64(
        q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z)}, ())


def _odom(msg, opts):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    return Sample(stamp_ns(msg), {'data': _f64(
        p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z)}, ())


def _twist_vals(t):
    return _f64(t.linear.x, t.linear.y, t.linear.z,
                t.angular.x, t.angular.y, t.angular.z)


def _twist(msg, opts):
    return Sample(-1, {'data': _twist_vals(msg)}, ())


def _twist_stamped(msg, opts):
    return Sample(stamp_ns(msg), {'data': _twist_vals(msg.twist)}, ())


def _transform_stamped(msg, opts):
    t, q = msg.transform.translation, msg.transform.rotation
    return Sample(stamp_ns(msg), {'data': _f64(
        t.x, t.y, t.z, q.x, q.y, q.z, q.w)}, ())


TfRow = namedtuple('TfRow', 'parent child stamp_ns data')
TF_COLUMNS = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']


def _frame_name(name):
    return str(name).lstrip('/')


def tf_transforms(msg, pairs=None):
    """
    Return one ``TfRow`` per transform in a TFMessage / tf_static message.

    ``pairs`` is None (keep all) or a collection of (parent, child) tuples;
    only direct edges that match are kept, so an untracked child is dropped.
    Leading ``/`` on frame names is stripped before matching. ``data`` is a
    float64 vector ``x y z qx qy qz qw`` (the TransformStamped layout).
    """
    keep = None if pairs is None else set(pairs)
    rows = []
    for t in msg.transforms:
        parent = _frame_name(t.header.frame_id)
        child = _frame_name(t.child_frame_id)
        if keep is not None and (parent, child) not in keep:
            continue
        s = stamp_ns(t)
        tr, q = t.transform.translation, t.transform.rotation
        rows.append(TfRow(parent, child, s,
                          _f64(tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)))
    return rows


def _tf_message(msg, opts):
    rows = tf_transforms(msg, opts.get('pairs'))
    data = (np.stack([r.data for r in rows]) if rows
            else np.zeros((0, 7), np.float64))
    return Sample(-1, {'data': data}, ('data',))


def _vector3_stamped(msg, opts):
    v = msg.vector
    return Sample(stamp_ns(msg), {'data': _f64(v.x, v.y, v.z)}, ())


def _cols(*names):
    return lambda msg, opts: {'columns': list(names)}


def _stamped_cols(*names, child=False):
    """Attrs for a stamped message: columns + frame_id (+ child_frame_id)."""
    def describe(msg, opts):
        a = {'columns': list(names), 'frame_id': _fid(msg.header.frame_id)}
        if child:
            a['child_frame_id'] = _fid(msg.child_frame_id)
        return a
    return describe


_NO_ATTRS = lambda msg, opts: {}   # noqa: E731

CONVERTERS = {
    'raw_radar_msgs/msg/ADCDataCube': Converter(
        _adc_cube, _adc_cube_attrs, 'reliable'),
    'raw_radar_msgs/msg/ADCChirpMeta': Converter(
        _adc_meta, _NO_ATTRS, 'reliable'),
    'sensor_msgs/msg/PointCloud2': Converter(_cloud, _cloud_attrs, 'best_effort'),
    'sensor_msgs/msg/Image': Converter(_image, _image_attrs, 'best_effort'),
    'sensor_msgs/msg/Imu': Converter(_imu, _stamped_cols(
        'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'),
        'best_effort'),
    'nav_msgs/msg/Odometry': Converter(_odom, _stamped_cols(
        'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        'vx', 'vy', 'vz', 'wx', 'wy', 'wz', child=True), 'best_effort'),
    'geometry_msgs/msg/Twist': Converter(_twist, _cols(
        'vx', 'vy', 'vz', 'wx', 'wy', 'wz'), 'best_effort'),
    'geometry_msgs/msg/TwistStamped': Converter(_twist_stamped, _stamped_cols(
        'vx', 'vy', 'vz', 'wx', 'wy', 'wz'), 'best_effort'),
    'geometry_msgs/msg/TransformStamped': Converter(_transform_stamped, _stamped_cols(
        'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', child=True), 'best_effort'),
    'tf2_msgs/msg/TFMessage': Converter(_tf_message, _cols(*TF_COLUMNS),
                                        'reliable'),
    'geometry_msgs/msg/Vector3Stamped': Converter(
        _vector3_stamped, _stamped_cols('x', 'y', 'z'), 'best_effort'),
}


def normalize_type(t):
    """'pkg/Name' or 'pkg/msg/Name' -> 'pkg/msg/Name'."""
    parts = t.strip().split('/')
    if len(parts) == 2:
        parts = [parts[0], 'msg', parts[1]]
    return '/'.join(parts)


def get_converter(msg_type):
    t = normalize_type(msg_type)
    if t not in CONVERTERS:
        raise KeyError('no converter for %s (have: %s)' %
                       (t, ', '.join(sorted(CONVERTERS))))
    return CONVERTERS[t]

# Reading a dataset_generator HDF5 capture

Reader guide: h5py + numpy only, no ROS needed. Layout, columns and versions are in
[hdf5_format.md](hdf5_format.md). Every block below runs against a file the real writer produced
(file name `capture.h5`; stream names are the preset names, e.g. `radar_pc`, `imu`).

## 1. Open, detect, iterate

```python
import h5py, numpy as np
with h5py.File('capture.h5') as f:
    ver = f.attrs['format_version']                    # detect newer features by attribute, not version
    ok = bool(f.attrs['closed_cleanly'])               # False: the recorder died; data up to the last flush
    for n in f:
        if n not in ('tf', 'tf_static'):               # those two groups are not streams
            print(n, f[n].attrs['msg_type'], f[n].attrs['n_samples'], f[n].attrs.get('frame_id'))
```

Every stream group has `stamp_header_ns` and `stamp_recv_ns` (int64, one per sample); sample `i` of every
dataset in a group is the same message. A stream that never delivered (`allow_missing`) has an empty group.
Attributes written as lists (`columns`, `axes`) come back as object arrays of `str`.

## 2. One example per stream type

Radar cube (`int16 (N, 2, rx, sample, chirp)`, axis 1 real/imag), chirp meta (no `data`; one dataset per field,
per-chirp ones `(N, n_chirps)`) and a ragged PointCloud2 (`data (sum(rows), K)` plus `row_count (N,)`, names in
`columns`). Radar specs: `json.loads(f['radar_adc'].attrs['radar_cfg_metrics'])`.

```python
import h5py, numpy as np
with h5py.File('capture.h5') as f:
    d = f['radar_adc/data'][3].astype(np.float32)
    iq = d[0] + 1j * d[1]                              # complex (rx, sample, chirp)
    print(f['radar_adc'].attrs['axes'], iq.shape)
    m = f['adc_meta']
    print(m['frame_index'][:], m['t_s'].shape, m['record_valid'][2].all())
    g = f['radar_pc']
    off = np.concatenate([[0], np.cumsum(g['row_count'][:])])     # sample i = data[off[i]:off[i+1]]
    pts = g['data'][off[2]:off[3]]
    cols = list(g.attrs['columns'])                    # ['x', 'y', 'z', 'vel']
    xyz, vel = pts[:, :3], pts[:, cols.index('vel')]
```

Image (`(N, H, W[, C])` in the message encoding, attr `encoding`), Imu / Odometry / Twist (float64 rows, names in
`columns`) and CameraInfo (`float64 (N, 46)`: `width height K0..K8 R0..R8 P0..P11 D0..D13`, D NaN-padded).
Quaternions are x-first. Odometry pose is in `frame_id`, twist in `child_frame_id`; Twist has no header
(`stamp_header_ns == -1`), so use `stamp_recv_ns`.

```python
import h5py, numpy as np
with h5py.File('capture.h5') as f:
    g = f['camera']
    img = g['data'][0]                                 # (H, W, 3) uint8
    bgr = img[..., ::-1] if g.attrs['encoding'] == 'rgb8' else img   # for OpenCV
    imu = f['imu']; c = list(imu.attrs['columns'])     # qx qy qz qw wx wy wz ax ay az
    gyro = imu['data'][:, [c.index(k) for k in ('wx', 'wy', 'wz')]]
    od = f['odom']; c = list(od.attrs['columns'])      # x y z qx qy qz qw vx vy vz wx wy wz
    xy = od['data'][:, [c.index('x'), c.index('y')]]
    print(od.attrs['frame_id'], od.attrs['child_frame_id'], f['cmd_vel/stamp_header_ns'][0])
    g = f['cam_info']; c = list(g.attrs['columns']); row = g['data'][0]
    K = row[[c.index('K%d' % i) for i in range(9)]].reshape(3, 3)
    D = row[[c.index('D%d' % i) for i in range(14)]]; D = D[~np.isnan(D)]
    print(K[0, 0], D, g.attrs['distortion_model'])
```

## 3. Time alignment across streams

`stamp_header_ns` is the sensor's clock (`-1` if unstamped); `stamp_recv_ns` is the recording host's clock,
common to all streams. Use header stamps when the drivers share a clock, else `stamp_recv_ns`. Index `i` of
one stream has no relation to index `i` of another.

```python
import h5py, numpy as np
with h5py.File('capture.h5') as f:
    t = f['radar_pc/stamp_header_ns'][:]
    ti = f['imu/stamp_header_ns'][:]
    j = np.clip(np.searchsorted(ti, t), 1, len(ti) - 1)
    near = j - ((t - ti[j - 1]) < (ti[j] - t))         # nearest IMU sample per cloud
    last = np.searchsorted(ti, t, side='right') - 1    # newest IMU at or before it; -1 = none yet
    win = [np.flatnonzero((ti > a) & (ti <= b)) for a, b in zip(t[:-1], t[1:])]  # IMU between clouds
    print(near, last, [len(w) for w in win])
```

## 4. Transforms and the tf tree

The file does **not** move data into the body frame. Clouds, radar, images and IMU stay in their sensor
frame (`frame_id` attr = the first message's `header.frame_id`, no leading `/`). Storage of `/tf_static`, the tracked `/tf/<parent>__<child>` pairs and the root attrs is in
[hdf5_format.md](hdf5_format.md) (Transforms). `robot_description` (root attr, URDF text) exists only if a
publisher existed; `<joint><origin xyz rpy>` mirrors the mounts, but `/tf_static` is authoritative.

Every transform is `T_parent_child`: it maps a point in `child` coordinates into `parent` coordinates.
Only `tf_track` edges are recorded over time; any other moving link is not in the file.

```mermaid
flowchart LR
    odom -- "/tf pair odom__base_link<br/>(dynamic, Rover odometry)" --> base_link
    base_link -- "/tf_static<br/>(mount)" --> radar["radar (cube / cloud frame_id)"]
```

Rover Pro radar preset; the radar frame name is the driver's `header.frame_id`. List your file's frames
with `f['tf_static/parent'].asstr()[:]` and `f['tf_static/child'].asstr()[:]`.

**Sensor to base frame (what v1 did).** v1 transformed point clouds (x, y, z only; `vel` and `intensity`
untouched) into a base frame (`base_footprint` or `base_level` on the UGVs/UAVs) using the newest tf in its
buffer at save time. For a fixed mount that equals the static chain, so `T_base_sensor` is the whole
answer. For a moving chain, add the tracked pair as below.

```python
import h5py, numpy as np

def T(xyz, q):                                         # 4x4 from x y z, qx qy qz qw
    x, y, z, w = np.asarray(q) / np.linalg.norm(q)
    M = np.eye(4)
    M[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                 [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]]
    M[:3, 3] = xyz
    return M

def static_chain(f, frame, up_to):                     # T_up_to_frame from /tf_static
    g = f['tf_static']
    parent, child = g['parent'].asstr()[:], g['child'].asstr()[:]
    M = np.eye(4)
    while frame != up_to:
        i = list(child).index(frame)                   # ValueError: no static path up to `up_to`
        M = T(g['xyz'][i], g['quat_xyzw'][i]) @ M
        frame = parent[i]
    return M

def pose_at(g, t_ns, interpolate=True):                # tracked group -> T_parent_child at t
    ts, d = g['stamp_header_ns'][:], g['data'][:]
    j = int(np.clip(np.searchsorted(ts, t_ns), 1, max(len(ts) - 1, 1)))
    a, b = max(j - 1, 0), min(j, len(ts) - 1)
    if not interpolate:                                # nearest sample
        a = a if abs(t_ns - ts[a]) <= abs(ts[b] - t_ns) else b
        return T(d[a, :3], d[a, 3:])
    u = 0.0 if ts[b] == ts[a] else float(np.clip((t_ns - ts[a]) / (ts[b] - ts[a]), 0, 1))
    qa, qb = d[a, 3:], d[b, 3:]
    qb = -qb if qa @ qb < 0 else qb                    # same hemisphere, then nlerp (small steps)
    return T((1-u) * d[a, :3] + u * d[b, :3], (1-u) * qa + u * qb)

with h5py.File('capture.h5') as f:
    g = f['radar_pc']
    off = np.concatenate([[0], np.cumsum(g['row_count'][:])])
    i = 2
    pts = g['data'][off[i]:off[i + 1]].astype(np.float64)       # x y z vel, sensor frame
    T_base_sensor = static_chain(f, g.attrs['frame_id'], 'base_footprint')
    xyz_base = pts[:, :3] @ T_base_sensor[:3, :3].T + T_base_sensor[:3, 3]
    v1_cloud = np.c_[xyz_base, pts[:, 3]].astype(np.float32)    # what v1 saved: (n, 4)
    # a world (odom) frame needs the dynamic edge at the cloud's stamp
    trk = f['tf/odom__base_link']
    T_odom_base = pose_at(trk, g['stamp_header_ns'][i])         # interpolate=False: nearest sample
    T_odom_sensor = T_odom_base @ static_chain(f, g.attrs['frame_id'], trk.attrs['child_frame'])
    xyz_odom = pts[:, :3] @ T_odom_sensor[:3, :3].T + T_odom_sensor[:3, 3]
    print(v1_cloud[0], xyz_odom[0])
```

The default interpolates at the sample's own stamp (clamped outside the tracked range). Match stamps on
`stamp_recv_ns` if the clocks differ. `pose_at` returns the tracked edge only.

## 5. Migrating a v1 `frame_N.npy` reader

v1 wrote `<dataset>/<folder>/frame_<N>.npy` (camera and depth `.png`), one file per folder per save tick,
`N` starting at 10000, all sensors in one tick. The HDF5 file replaces the folder tree with one group per
stream and one row per message.

| v1 folder / file | HDF5 group (preset) and read | Difference |
|---|---|---|
| `<radar>_adc`: complex128 `(rx, sample, chirp)` | `radar_adc/data`: `d[:,0] + 1j*d[:,1]` | every cube is kept (reliable QoS); per-sample stamps; cfg text in attrs; `adc_meta` per-chirp info |
| `<radar>_pc`: float32 `(n, 4)` x y z vel, **base frame** | `radar_pc`: ragged slice, **sensor frame** | apply `T_base_sensor` (section 4); NaN rows dropped as before |
| `lidar`: float32 `(n, 4)` x y z intensity, base frame | `lidar` ragged | as above; columns follow `columns` (set `fields` to match) |
| `camera`, `depth`: `.png` (BGR / as published) | `camera/data[i]`, `camera_depth/data[i]` | message encoding (`encoding` attr); write PNGs yourself |
| `hands`: `(2, n, 3)` | `hand_left`, `hand_right` ragged | two streams, not transformed; `np.stack` yourself, counts may differ |
| `imu_data`: `(m, 7)` t wx wy wz ax ay az | `imu/data` cols `wx wy wz ax ay az` | one row per message (v1 repeated the latest row); has orientation; t is `stamp_header_ns` ns int64, not float seconds |
| `vehicle_odom`: `(m, 14)` t x y z **qw qx qy qz** vx vy vz wx wy wz | `odom/data`: x y z **qx qy qz qw** vx vy vz wx wy wz | quaternion reorder `[0,1,2,6,3,4,5,7,8,9,10,11,12]`; no time column |
| `vehicle_vel`: `(m, 3)` t vx wz | none | `odom` columns `vx`, `wz` |
| `vicon_<segment>`: `(7,)` x y z **qw qx qy qz** | a `TransformStamped` stream, `data (N, 7)` x y z **qx qy qz qw** | reorder |
| (none) | `/tf_static`, `/tf/*`, `robot_description`, `adc_meta`, `cam_info` | new |

Recipe: rebuild a v1 tick from the file by taking, for each stream, the newest sample at or before the
tick time (v1 saved the latest message per sensor).

```python
import h5py, numpy as np

def latest(g, t, key='stamp_recv_ns'):                 # index of newest sample <= t, -1 if none
    return int(np.searchsorted(g[key][:], t, side='right')) - 1

with h5py.File('capture.h5') as f:
    tick = f['radar_pc/stamp_recv_ns'][2]              # v1 tick time: any host-clock instant
    imu, od = f['imu'], f['odom']
    t0 = tick - 100_000_000                            # one low-rate period of IMU history
    sel = np.flatnonzero((imu['stamp_header_ns'][:] > t0) & (imu['stamp_header_ns'][:] <= tick))
    v1_imu = np.c_[imu['stamp_header_ns'][sel] * 1e-9, imu['data'][sel][:, 4:10]]   # t wx wy wz ax ay az
    k = np.flatnonzero((od['stamp_header_ns'][:] > t0) & (od['stamp_header_ns'][:] <= tick))
    q_wfirst = [0, 1, 2, 6, 3, 4, 5, 7, 8, 9, 10, 11, 12]
    v1_odom = np.c_[od['stamp_header_ns'][k] * 1e-9, od['data'][k][:, q_wfirst]]    # t x y z qw qx qy qz v w
    v1_vel = v1_odom[:, [0, 8, 13]]                    # t vx wz
    j = latest(f['radar_adc'], tick)                   # then iq as in section 2
    print(v1_imu.shape, v1_odom.shape, v1_vel[0], j)
```

Behavioural differences:

- **Frame.** v1 clouds were in `base_footprint` (UGV, UAV) or `base_level` (some UAV presets), not the
  `base_link` the recorder tracks. `static_chain` reaches any frame `/tf_static` carries; list them first.
- **Time.** v1 stored no header stamp (column 0 of IMU/odom was the node clock at buffer time). Now every
  sample has both stamps; align per section 3, or use `capture_mode: sampled` (newest unseen message per tick).
- **Numbering and ticks.** v1 files were `frame_10000`, ...; here rows start at 0. v1 skipped a tick until every
  sensor had data and emptied its folders at start; now the start gate waits once and each capture is a new `.h5`.
- **Export tool.** `ros2 run dataset_generator hdf5_to_legacy capture.h5 out_dir [--reference STREAM]` writes
  `frame_<idx>.npy` from 0, one per sample of the reference stream, with the newest sample of each other stream
  received at or before it. The default reference is the first stream in h5py order, **alphabetical** (not YAML
  order), so pass `--reference`. Output is sensor-frame, x-first quaternions, one latest IMU/odom row, no
  `vehicle_vel`, no PNG.

# dataset_generator v2: HDF5 capture format

`ros2 launch dataset_generator record_dataset.launch.py param_file:=radar_sar.yaml [output_file:=/path/x.h5]`
runs the `dataset_recorder` node (any YAML with a `streams:` table). YAMLs without
one (`ugv_dataset_v1.yaml`, `uav_*.yaml`, `human_movement.yaml`) still launch the v1
`dataset_generator` node, which writes the old folder-of-.npy layout. `ugv_dataset.yaml`
is now v2; its v1 form is `ugv_dataset_v1.yaml`.

One HDF5 file per capture (default `<dataset_path>/capture_<YYYYmmdd_HHMMSS>.h5`).

## YAML

```yaml
dataset_generator:
  ros__parameters:
    output_file: ""        # full path; launch arg output_file overrides
    dataset_path: "..."    # used when output_file is empty
    capture_mode: all      # all: every message | sampled: newest unseen message per tick
    sample_rate_hz: 10.0   # sampled-mode tick rate
    flush_period_s: 5.0
    duration_s: 0.0        # >0: stop and close cleanly after this long
    queue_size: 256        # writer-thread queue (samples); overflow is counted, not blocking
    streams:
      <name>:              # HDF5 group name (no '.' or '/')
        topic: radar_0/adc_data_cube
        type: raw_radar_msgs/msg/ADCDataCube
        enabled: true
        qos: reliable      # reliable | best_effort; default per type (below)
        qos_depth: 10      # reliable is raised to >= 10
        mode: all          # per-stream override of capture_mode
        fields: [x,y,z,vel]  # PointCloud2 columns (default: all fields)
        skip_nans: true      # PointCloud2
        compression: gzip    # optional (gzip | lzf), any stream
        legacy_folder: radar_0_adc  # folder name used by hdf5_to_legacy
        meta_for: radar_adc  # on an ADCChirpMeta stream: the cube stream to cross-check
```

Supported `type`s: `raw_radar_msgs/ADCDataCube`, `raw_radar_msgs/ADCChirpMeta`,
`sensor_msgs/{PointCloud2,Image,Imu}`, `nav_msgs/Odometry`,
`geometry_msgs/{Twist,TwistStamped,TransformStamped,Vector3Stamped}`
(`converters.py`; add one function + a table row for a new type).

QoS defaults: ADCDataCube and ADCChirpMeta `reliable`; all others `best_effort`
(a best-effort subscriber matches any publisher). The ADC cube MUST be reliable: best-effort
lost about half of the 3.4 MB SAR frames (running/radar-03_step2_rate_diag.log, in the console repo).

## Layout

Root attrs: `format="cpsl_dataset_hdf5"`, `format_version=2`, `closed_cleanly` (False
until a clean close; False means the node died), `start_recv_ns`, `end_recv_ns`,
`last_flush_recv_ns`, `created_utc`, `host`, `ros_node`, `capture_mode`, `sample_rate_hz`,
`config_streams` (the stream table as JSON), `timestamps` (field meaning).

Per stream, group `/<name>`:

| dataset | dtype / shape | meaning |
|---|---|---|
| `stamp_header_ns` | int64 (N,) | message header stamp, ns (`-1` for unstamped, e.g. Twist) |
| `stamp_recv_ns` | int64 (N,) | host `time.time_ns()` when the node received it |
| `data` | per type, (N, ...) | the payload, one row per sample, chunked + resizable |
| other fields | (N, ...) | ADCChirpMeta only: one dataset per message field |
| `row_count` | int64 (N,) | ragged streams (PointCloud2) only: rows of `data` per sample; sample `i` is `data[cumsum[i-1]:cumsum[i]]` |

`data` by type: ADCDataCube `int16 (N, 2, rx, sample, chirp)` with index 0 = real, 1 = imag
(complex = `d[:,0] + 1j*d[:,1]`; axis labels in attr `axes`); PointCloud2 `float32 (rows, len(fields))`
(`float64` if a field is double; attr `columns`); Image `(N, H, W[, C])` in the message's encoding
(attr `encoding`); Imu 10 cols, Odometry 13, Twist 6, TransformStamped 7, Vector3 3
(attr `columns` names them). ADCChirpMeta: scalars `(N,)` and per-chirp arrays `(N, n_chirps)`
named as in the .msg (`frame_index`, `record_valid`, `t_s`, ..., `prev_tail_sat (N,2)`).

Stream attrs: `topic`, `msg_type`, `qos`, `qos_depth`, `mode`, `n_samples`, `n_errors`
(samples rejected by the writer, e.g. a shape change), `n_dropped_node` (converter errors +
queue overflow), and for any stream with a `frame_index` field `frame_index_first/last`,
`frame_index_recorded_gaps` (missing indices between consecutive recorded messages, uint32
wrap-safe) and `frame_index_nonmonotonic`.

Capture metadata for lost ADC frames (written at close when `meta_for` is set; `adc_meta`
must be published, i.e. the driver ran with `publish_chirp_meta`): on the cube stream
`frame_index_published` (span first..last of the meta stream), `frame_index_recorded`,
`frame_index_gap = published - recorded`, `n_without_meta`; on the meta stream `n_without_cube`.
Cubes and metas are matched on header stamp. The published count is a lower bound if the meta
topic itself lost messages. In `sampled` mode these gaps are expected.

The file is flushed every `flush_period_s` and closed on SIGINT/SIGTERM or `duration_s`.
A killed process leaves an HDF5 file that may need recovery; check `closed_cleanly`.

## Reading

```python
import h5py, numpy as np
with h5py.File("capture.h5") as f:
    cube = f["radar_adc/data"][3]; iq = cube[0] + 1j * cube[1]
    latency_ns = f["radar_adc/stamp_recv_ns"][:] - f["radar_adc/stamp_header_ns"][:]
```

## Old-format compatibility

`ros2 run dataset_generator hdf5_to_legacy capture.h5 out_dir [--reference STREAM]` writes
`out_dir/<legacy_folder or stream>/frame_<idx>.npy`, one index per sample of the reference
stream (default first); other streams contribute their newest sample received at or before it,
which is the v1 "latest message" rule. ADC cubes become complex `[rx, sample, chirp]`.
Differences from v1: no base-frame transform of point clouds, camera frames are `.npy` not
PNG, IMU/odometry are the single latest sample, not the buffered high-rate window.

`h5py` is a runtime dependency (`python3-h5py`, declared as `exec_depend` in package.xml).

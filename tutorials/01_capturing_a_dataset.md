# Tutorial 01: Capturing a dataset

Goal: record sensor topics into one HDF5 file with the `dataset_recorder` node, then open it in Python. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md) (installed, built, sourced). The recorder works on any topics, so steps 1-4 can be tried with a fake publisher if you have no sensor yet.

---

## 1. Write a recording config

A config is a YAML file in `src/dataset_generator/configs/` with a `streams:` table, one entry per topic to record. The shipped ones are per setup and expect their sensors, for example `ugv_dataset.yaml` lists radar streams, so with no radar attached the start gate (step 3) never opens. Use the one below for a RealSense, or copy a shipped one and set `enabled: false` on what you do not have (or `allow_missing: true`). Save it as `src/dataset_generator/configs/my_capture.yaml`:

```yaml
dataset_generator:
  ros__parameters:
    dataset_path: "/home/you/datasets"   # an existing, writable folder
    capture_mode: "all"        # all = every message; sampled = latest per tick
    tf_track: [""]             # [""] = record no /tf pairs
    streams:
      camera:
        topic: "camera/camera/color/image_raw"   # relative to the recorder's namespace
        type: "sensor_msgs/msg/Image"
        compression: gzip
      camera_info:             # the camera's intrinsics, recorded at a low rate
        topic: "camera/camera/color/camera_info"
        type: "sensor_msgs/msg/CameraInfo"
        mode: sampled
```

`topic` and `type` are the only required keys per stream; every other key is listed in [`hdf5_format.md`](../src/dataset_generator/docs/hdf5_format.md). `tf_track` lists `"parent->child"` coordinate-frame pairs to record from `/tf` over time (a moving robot's pose); the file always also keeps `/tf_static`, the fixed sensor mounting transforms from the robot description. No camera? Replace both streams with an `imu` stream (`topic: "livox/imu"`, `type: "sensor_msgs/msg/Imu"`) and see step 2 for a fake publisher.

A new file must be installed once: `colcon build --packages-select dataset_generator --symlink-install`. Edits to an already installed file need no rebuild.

## 2. Start the sensors, then the recorder

Terminal 1, a bringup for your hardware (source both setup files first):

```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py realsense_enable:=true
```

Terminal 2, the recorder in the same namespace as the sensors (`default_template` here):

```bash
ros2 launch dataset_generator record_dataset.launch.py \
    namespace:=default_template param_file:=my_capture.yaml dataset_subpath:=run_001
```

`param_file` is a bare file name from `configs/`. `dataset_subpath` is appended to `dataset_path`. `output_file:=/full/path.h5` overrides both. No sensor? With the `imu` stream above, run `ros2 topic pub -r 10 /ns1/livox/imu sensor_msgs/msg/Imu "{}"` in a third terminal and use `namespace:=ns1`.

## 3. The start gate

The recorder writes nothing until **every enabled stream** has delivered a message, then logs `GATE open`. If one never arrives within `start_timeout_s` (30 s) it logs `GATE abort missing=<topics>`, creates no file and exits with code 2. To record what is there anyway, set `allow_missing: true`; silent streams are listed in the file's `streams_missing` attribute. Details: `hdf5_format.md`, section "Start gate".

## 4. Stop and read the file

Stop the recorder with `Ctrl-C`; it closes the file cleanly. The file is `<dataset_path>/<dataset_subpath>/capture_<time>.h5`:

```python
import glob, h5py
f = h5py.File(sorted(glob.glob("/home/you/datasets/run_001/*.h5"))[-1])
print(list(f.keys()), f.attrs["format_version"], f.attrs["closed_cleanly"])
print(f["camera/data"].shape, f["camera"].attrs["resolved_topic"])   # imu/data for the IMU file
```

You should see your stream names plus `tf_static`, `4 True`, and for an `rgb8` camera a shape `(N, H, W, 3)` with the absolute topic it listened to. An IMU stream is `(N, 10)`: orientation `x y z w`, angular velocity `x y z`, linear acceleration `x y z` per sample. Each stream group holds `data`, `stamp_header_ns` (the sender's stamp) and `stamp_recv_ns` (arrival time). `closed_cleanly` is `False` if the node was killed rather than stopped. A `camera_info` stream is an `(N, 46)` array (layout in `hdf5_format.md`). To read each stream type, move data into the robot's base frame or align streams in time, see [`reading_datasets.md`](../src/dataset_generator/docs/reading_datasets.md).

## 5. Check before a long recording

```bash
ros2 topic hz /default_template/camera/camera/color/image_raw   # your real topic
df -h /home/you/datasets                                       # free disk
```

Radar ADC cubes are several MB per frame; use `qos: reliable` for them as `ugv_dataset.yaml` does.

## Legacy: the old `.npy` folders

Older tools expect one folder per sensor with `frame_<n>.npy` files. Convert a capture (sample-aligned to the first stream, or `--reference <stream>`):

```bash
ros2 run dataset_generator hdf5_to_legacy capture.h5 out_dir
```

You should see a count per stream printed, and `out_dir/<legacy_folder or stream>/frame_0.npy`. Differences from the old recorder (no base-frame transform, camera frames as `.npy`) are listed in [`reading_datasets.md`](../src/dataset_generator/docs/reading_datasets.md), section 5 (clouds stay in the sensor frame there; the transform is in the file). Pass `--reference STREAM` to choose the stream that sets the frame count. The old recorder itself is gone; configs without a `streams:` table are rejected with an error.

# Tutorial 01: Capturing a dataset

Goal: record sensor topics into one HDF5 file with the `dataset_recorder` node, then open it in Python. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md) (installed, built, sourced). The recorder works on any topics, so steps 1-4 can be tried with a fake publisher if you have no sensor yet.

---

## 1. Pick or write a recording config

A config is a YAML file in `src/dataset_generator/configs/` with a `streams:` table, one entry per topic to record. The shipped ones are per setup (`ugv_dataset.yaml`, `radar_sar.yaml`, the `uav_*` and `human_movement` paper setups, see [Tutorial 08](08_paper_setups.md)). Copy one and keep only what you have:

```bash
cp src/dataset_generator/configs/ugv_dataset.yaml src/dataset_generator/configs/my_capture.yaml
```

In `my_capture.yaml`, set `dataset_path` to an existing, writable folder, and for each stream either point `topic:` at a topic that exists or set `enabled: false`. Topics are relative to the recorder's namespace. A minimal one-stream file:

```yaml
dataset_generator:
  ros__parameters:
    dataset_path: "/home/you/datasets"
    capture_mode: "all"        # all = every message; sampled = latest per tick
    tf_track: [""]             # no /tf pairs
    streams:
      imu:
        topic: "livox/imu"
        type: "sensor_msgs/msg/Imu"
```

`camera_info` is an optional `sensor_msgs/msg/CameraInfo` stream (camera intrinsics); the shipped configs list it with `enabled: false`, set it to `true` to record it. Every key is explained in [`hdf5_format.md`](../src/dataset_generator/docs/hdf5_format.md).

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

`param_file` is a bare file name from `configs/`. `dataset_subpath` is appended to `dataset_path`. `output_file:=/full/path.h5` overrides both. No fake sensor? Run `ros2 topic pub -r 10 /ns1/livox/imu sensor_msgs/msg/Imu "{}"` in a third terminal and use `namespace:=ns1` for the one-stream file above.

## 3. The start gate

The recorder writes nothing until **every enabled stream** has delivered a message, then logs `GATE open`. If one never arrives within `start_timeout_s` (30 s) it logs `GATE abort missing=<topics>`, creates no file and exits with code 2. To record what is there anyway, set `allow_missing: true`; silent streams are listed in the file's `streams_missing` attribute. Details: `hdf5_format.md`, section "Start gate".

## 4. Stop and read the file

Stop the recorder with `Ctrl-C`; it closes the file cleanly. The file is `<dataset_path>/<dataset_subpath>/capture_<time>.h5`:

```python
import glob, h5py
f = h5py.File(sorted(glob.glob("/home/you/datasets/run_001/*.h5"))[-1])
print(list(f.keys()), f.attrs["format_version"], f.attrs["closed_cleanly"])
print(f["imu/data"].shape, f["imu"].attrs["resolved_topic"])
```

You should see the stream names plus `tf_static`, `4 True`, and a shape `(N, 10)` for the IMU with the absolute topic it listened to. Each stream group holds `data`, `stamp_header_ns` (the sender's stamp) and `stamp_recv_ns` (arrival time). `closed_cleanly` is `False` if the node was killed rather than stopped. A `camera_info` stream is an `(N, 46)` array (layout in `hdf5_format.md`).

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

You should see a count per stream printed, and `out_dir/<legacy_folder or stream>/frame_0.npy`. Differences from the old recorder (no base-frame transform, camera frames as `.npy`) are listed in `hdf5_format.md`, section "Old-format compatibility". The old recorder itself is gone; configs without a `streams:` table are rejected with an error.

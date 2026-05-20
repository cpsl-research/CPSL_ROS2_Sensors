# Tutorial 01: Capturing a Dataset

This tutorial walks through recording a synchronized multi-sensor dataset from start to finish. No prior reading required.

---

## 1. Choosing the Right Bringup Launch File

Select the launch file that matches your platform and sensor hardware:

| Platform | Launch File | Default Sensors |
|----------|-------------|-----------------|
| UGV with Livox + 2× IWR1843 radars | `ugv_sensor_bringup.launch.py` | Livox lidar, radar_0, radar_1, USB camera |
| UGV with Livox + 2× IWR1843 (RaGNNarok config) | `ugv_sensor_bringup_ragnnarok.launch.py` | Livox lidar, radar_0, radar_1, USB camera |
| UAV with Livox + 3× radars (RadSAR config) | `uav_sensor_bringup_radsar.launch.py` | platform description only; enable sensors with flags |
| UAV with Livox + 3× radars (IcaRAus config) | `uav_sensor_bringup_IcaRAus.launch.py` | platform description only; enable sensors with flags |
| UGV/standalone with Ouster LiDAR | `ouster_lidar_bringup.launch.py` | Ouster lidar |
| Human movement (RealSense + Leap Motion) | `human_movement_sensor_bringup.launch.py` | RealSense, Leap Motion, platform description |
| Quick single-radar test | `sensor_bringup_single_radar.launch.py` | Livox lidar, radar_0, USB camera |

All bringup launch files accept boolean `*_enable` flags to turn individual sensors on or off. For example:
```bash
# Start UGV bringup but skip the camera:
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py camera_enable:=false
```

---

## 2. Creating or Editing a Dataset YAML

Dataset configs live in `src/dataset_generator/configs/`. Working directory for editing: the **workspace root** (`CPSL_ROS2_Sensors/`).

Copy an existing config as a starting point:
```bash
# From workspace root:
cp src/dataset_generator/configs/ugv_dataset.yaml \
   src/dataset_generator/configs/my_experiment.yaml
```

Annotated example (based on `ugv_dataset.yaml`):

```yaml
dataset_generator:
  ros__parameters:

    # --- Sensor enable/disable ---
    radar_enable: True          # record radar point clouds
    lidar_enable: False         # record Livox/Ouster point cloud
    lidar_topic: "livox/lidar"  # topic name within the namespace

    camera_enable: True                          # record USB camera
    camera_topic: "/cpsl_ugv_1/image_raw"        # full topic path (namespaced)

    camera_depth_enable: False
    camera_depth_topic: "/cpsl_ugv_1/camera/cpsl_realsense/depth/image_rect_raw"

    hand_tracking_enable: False
    hand_tracking_left_topic: "/cpsl_human_movement/left_hand_joints"
    hand_tracking_right_topic: "/cpsl_human_movement/right_hand_joints"

    imu_enable: False
    imu_topic: "livox/imu"

    vicon_enable: False          # record Vicon ground truth

    vehicle_odom_enable: True
    vehicle_odom_topic: "odom_repub"   # relative to namespace

    # --- Reference frame ---
    # Must match the base_link name in the active URDF.
    # Format: "<namespace>/base_footprint"
    base_frame: "cpsl_ugv_1/base_footprint"

    # --- Recording rates ---
    frame_rate_save_data: 10.0          # Hz — rate at which frames are saved to disk
    frame_rate_high_speed_sensors: 50.0 # Hz — internal acquisition rate for fast sensors

    # --- Output path ---
    # dataset_subpath (from launch arg) is appended to this path at runtime.
    dataset_path: "/home/cpsl/Downloads/datasets/my_experiment"
```

Topic names that are relative (no leading `/`) are resolved within the node's namespace. Absolute paths (leading `/`) are used as-is.

---

## 3. Launching the Recorder

The recorder is a separate launch from the sensor bringup. Run them in two separate terminals.

**Terminal 1 — sensors:**
```bash
# From workspace root:
source install/setup.$(basename $SHELL)
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py
```

**Terminal 2 — recorder:**
```bash
# From workspace root:
source install/setup.$(basename $SHELL)
ros2 launch dataset_generator record_dataset.launch.py \
    param_file:=my_experiment.yaml \
    dataset_subpath:=run_001
```

Key arguments:

| Argument | Description |
|----------|-------------|
| `param_file` | Bare filename only (e.g. `my_experiment.yaml`). The launch file resolves it to the installed `configs/` directory automatically — do not provide a full path. |
| `dataset_subpath` | Appended to `dataset_path` from the YAML. Useful for numbering runs without editing the YAML each time. |
| `namespace` | Match the namespace used by the bringup launch (default: empty). |

The final output path is: `<dataset_path>/<dataset_subpath>/`

---

## 4. Verifying Output

After recording, the output directory has this structure:

```
<dataset_path>/<dataset_subpath>/
├── radar_0/
│   ├── frame_000000.npy
│   ├── frame_000001.npy
│   └── ...
├── radar_1/
│   └── ...
├── lidar/
│   └── ...
├── camera/
│   └── ...
├── vehicle_odom/
│   └── ...
└── timestamps.npy      # per-frame wall-clock timestamps
```

- Files are named `frame_NNNNNN.npy` (zero-padded, six digits).
- Each `.npy` file is a NumPy array. Load with `np.load('frame_000000.npy', allow_pickle=True)`.
- `timestamps.npy` contains the ROS wall-clock time for each saved frame.

Quick sanity checks:
```bash
# Count frames recorded across sensors:
ls <dataset_path>/<dataset_subpath>/radar_0/ | wc -l
ls <dataset_path>/<dataset_subpath>/lidar/   | wc -l

# They should be equal (synchronized save rate).
```

---

## 5. Pre-Recording Checklist

Before starting the recorder, confirm each sensor is publishing at its expected rate:

```bash
# Check lidar (expect ~10–20 Hz):
ros2 topic hz /livox/lidar

# Check radar (expect ~10 Hz):
ros2 topic hz /radar_0/ti_radar

# Check camera (expect ~10–30 Hz):
ros2 topic hz /image_raw

# Check odometry:
ros2 topic hz /odom_repub
```

Healthy output looks like:
```
average rate: 10.021
    min: 0.095s  max: 0.106s  std dev: 0.00234s  window: 50
```

Warning signs:
- Rate much lower than expected → check hardware connection or driver logs
- `min` and `max` wildly different → sensor is dropping packets (check network buffer, rmem_max for radar)
- No output at all → topic name mismatch; run `ros2 topic list` to verify the actual topic names

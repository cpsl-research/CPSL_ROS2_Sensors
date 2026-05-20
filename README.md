# CPSL ROS2 Sensors

Multi-sensor ROS2 platform integrating TI radar, Livox/Ouster LiDAR, depth cameras, Leap Motion hand tracking, and Vicon motion capture for UGV, UAV, and human-movement research.

---

## Quick-Start

```bash
# From any directory:
git clone https://github.com/cpsl-research/CPSL_ROS2_Sensors
cd CPSL_ROS2_Sensors

# Install dependencies, clone sensor submodules, and build:
bash scripts/install.sh --sensors radar,livox   # choose the sensors you have

# Source the workspace (repeat in every new terminal):
source install/setup.$(basename $SHELL)
```

`install.sh` clones only the submodules needed for the sensors you specify — unneeded sensor repos are never downloaded.

| Flag | Argument | Default | Description |
|------|----------|---------|-------------|
| `--sensors` | comma-separated | `radar,livox` | Which sensors to install. Valid values: `radar`, `livox`, `ouster`, `realsense`, `leapmotion`, `vicon`. Controls which submodules are cloned and which SDK dependencies are installed. |
| `--livox-ip` | `XX` (two digits) | _(none)_ | Last two digits of the Livox Mid360 serial number. Auto-patches `host_ip` in `MID360_config.json`. |
| `--ouster-hostname` | hostname or IP | _(none)_ | Auto-patches `sensor_hostname` in `ouster_configs/driver_params.yaml`. |
| `--skip-build` | _(flag)_ | off | Skip the colcon build step. Useful for setting up dependencies before hardware is connected. |

If ROS2 Jazzy is not yet installed, run `bash scripts/install_ros2.sh` instead — it installs ROS2 first, then hands off to `install.sh`.

---

## Sensor Support

| Sensor | Package / Submodule | Topics Published | Notes |
|--------|---------------------|------------------|-------|
| TI Radar (serial) | `CPSL_TI_Radar_ROS2` → `ti_radar_connect` | `<ns>/radar_N/ti_radar` (point cloud), `<ns>/radar_N/ti_radar_scan` | IWR1843 / IWR6843 via `/dev/ttyACM*` or `/dev/ttyUSB*` |
| TI Radar (DCA1000) | `CPSL_TI_Radar_ROS2` → `ti_radar_connect` | same as serial | Ethernet capture; requires IP config (see Hardware Setup) |
| Livox Mid360 | `CPSL_ROS_livox_ros_driver2` | `<ns>/livox/lidar` (point cloud), `<ns>/livox/imu` | Static-IP ethernet; requires `MID360_config.json` update |
| Ouster | `ouster-ros` | `<ns>/ouster/points`, `<ns>/ouster/imu`, `<ns>/ouster/scan` | Link-local ethernet; requires Eclipse Cyclone DDS |
| Intel RealSense | `realsense2_camera` (apt) | `<ns>/cpsl_realsense/color/image_raw`, `.../depth/image_rect_raw` | USB3; plug and play after apt install |
| Leap Motion | `CPSL_ROS2_LeapMotion` | `<ns>/left_hand_joints`, `<ns>/right_hand_joints`, `<ns>/leapmotion/image` | Requires Ultraleap Gemini service running |
| Vicon | `ros2-vicon-bridge` | `/tf` (object poses keyed by `vicon/<object>`) | Connects to Vicon DataStream over TCP |

---

## Hardware Setup

### TI Radar — Serial (IWR1843 / IWR6843)

1. Add yourself to the `dialout` group (the install script does this automatically):
   ```bash
   sudo usermod -a -G dialout $USER
   # log out and back in
   ```
2. Port order when multiple radars are connected:
   - IWR1843: `/dev/ttyACM0` = front radar (config), `/dev/ttyACM1` = front radar (data); `/dev/ttyACM2` = back, `/dev/ttyACM3` = back data
   - IWR6843: `/dev/ttyUSB0` (config), `/dev/ttyUSB1` (data)
3. Edit the JSON config in `src/CPSL_TI_Radar_ROS2/.../configs/` to match the assigned ports.

### TI Radar — DCA1000 Ethernet Capture

**Single radar (default):**
- Set your PC ethernet adapter to static `192.168.33.30/24`
- DCA1000 factory FPGA IP: `192.168.33.180`

**Multi-radar or co-located with Livox Mid360:**
- Both share one adapter; reprogram DCA1000 EEPROM to the `192.168.1.x` subnet using the CLI tool in `src/CPSL_TI_Radar_ROS2/DCA_Programming/`
- PC adapter: `192.168.1.57/24`
- Front DCA1000: `192.168.1.180` (port pair `4096/4098`)
- Back DCA1000: `192.168.1.182` (port pair `4088/4090`)

**Required kernel tuning (apply once, make permanent):**
```bash
sudo sysctl -w net.core.rmem_max=134217728
# permanent:
echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/99-radar.conf
sudo sysctl --system
```

### Livox Mid360

1. Set your PC ethernet adapter to static `192.168.1.XX/24`, where `XX` are the last two digits of the sensor's serial number.
2. Update `src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`:
   ```json
   "host_net_info": { "host_ip": "192.168.1.XX", ... }
   ```
   Or pass `--livox-ip XX` to `install.sh` to patch it automatically.

### Ouster

1. Set your PC ethernet adapter to link-local: IP `169.254.1.1`, netmask `255.255.0.0`.
2. Set the sensor hostname in `src/cpsl_ros2_sensors_bringup/ouster_configs/driver_params.yaml`:
   ```yaml
   sensor_hostname: 'os-XXXXXXXXXXXX.local'   # or IP address
   ```
   Or pass `--ouster-hostname <host>` to `install.sh` to patch it automatically.
3. Eclipse Cyclone DDS is required; the install script appends `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to your shell RC file.

---

## Launch Reference

All launch files live in `src/cpsl_ros2_sensors_bringup/launch/`. Launch with:
```bash
ros2 launch cpsl_ros2_sensors_bringup <file> [arg:=value ...]
```

| Launch File | Default Namespace | Default Sensors On | Default Radar Configs |
|-------------|-------------------|--------------------|-----------------------|
| `ugv_sensor_bringup.launch.py` | _(empty)_ | lidar, radar (×2), camera | `radar_0_IWR1843_vel_sr.json` / `radar_1_IWR1843_vel_sr.json` |
| `ugv_sensor_bringup_ragnnarok.launch.py` | _(empty)_ | lidar, radar (×2), camera | `front_radar_IWR1843_RaGNNarok_UGV_5m.json` / `back_radar_IWR1843_RaGNNarok_UGV_5m.json` |
| `uav_sensor_bringup_radsar.launch.py` | `cpsl_uav_1` | platform description only | `front/back_radar_IWR1843_dca_RadVel_10Hz.json`, `down_radar_IWR6843_ods_dca_RadVel.json` |
| `uav_sensor_bringup_IcaRAus.launch.py` | `cpsl_uav_1` | platform description only | `front/back_radar_IWR1843_IcaRAus.json`, `down_radar_6843_IcaRAus_ods_10Hz.json` |
| `ouster_lidar_bringup.launch.py` | _(empty)_ | ouster lidar | — |
| `human_movement_sensor_bringup.launch.py` | `cpsl_human_movement` | realsense, leapmotion, platform | `radar_0_IWR6843_ods_human_movement.json` |
| `sensor_bringup_single_radar.launch.py` | _(empty)_ | lidar, radar (×1), camera | `radar_0_IWR1843_vel_sr.json` |

**Common arguments** accepted by most launch files:

| Argument | Values | Description |
|----------|--------|-------------|
| `namespace` | string | Scopes all node names and topics |
| `lidar_enable` | `true`/`false` | Start Livox (or Ouster) driver |
| `radar_enable` | `true`/`false` | Start TI radar driver(s) |
| `camera_enable` | `true`/`false` | Start USB camera node |
| `platform_description_enable` | `true`/`false` | Publish URDF + static TF |
| `lidar_scan_enable` | `true`/`false` | Also publish a `/scan` LaserScan |
| `rviz` | `true`/`false` | Open an RViz window |

UAV launch files replace the single `radar_enable` flag with per-radar flags: `front_radar_enable`, `back_radar_enable`, `down_radar_enable` — each with a matching `*_config_file` argument.

---

## Dataset Recording

Dataset configs live in `src/dataset_generator/configs/*.yaml`. Launch the recorder alongside a bringup launch:

```bash
# In terminal 1 — start sensors:
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py

# In terminal 2 — start recording:
ros2 launch dataset_generator record_dataset.launch.py \
    param_file:=ugv_dataset.yaml \
    dataset_subpath:=run_001
```

- `param_file` — bare filename (no path); resolved inside the installed `configs/` directory
- `dataset_subpath` — appended to `dataset_path` from the YAML; useful for numbered runs
- Output path: `<dataset_path>/<dataset_subpath>/frame_XXXXXX.npy` with per-sensor subdirectories

See [`tutorials/01_capturing_a_dataset.md`](tutorials/01_capturing_a_dataset.md) for a full walkthrough.

---

## Further Reading

- [`tutorials/01_capturing_a_dataset.md`](tutorials/01_capturing_a_dataset.md) — end-to-end dataset recording guide
- [`tutorials/02_adding_a_new_platform.md`](tutorials/02_adding_a_new_platform.md) — add a new robot URDF and wire it into bringup
- [`tutorials/03_writing_a_bringup_launch_file.md`](tutorials/03_writing_a_bringup_launch_file.md) — write a new bringup launch file from scratch
- [`src/CPSL_TI_Radar_ROS2/README.md`](src/CPSL_TI_Radar_ROS2/README.md) — TI radar driver details and config reference
- [`src/CPSL_ROS_livox_ros_driver2/README.md`](src/CPSL_ROS_livox_ros_driver2/README.md) — Livox driver details
- [`src/ouster-ros/README.md`](src/ouster-ros/README.md) — Ouster driver details

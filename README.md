# CPSL ROS2 Sensors

Multi-sensor ROS 2 Jazzy workspace for CPSL research platforms (ground robots, UAVs, human-movement rigs): TI radar, Livox and Ouster lidar, RealSense and USB cameras, Leap Motion hand tracking and Vicon, with launch files that start them together and an HDF5 recorder that captures their topics. It stands alone: you need only Ubuntu 24.04 and this repo. [console](https://github.com/cpsl-research/console) is an optional browser GUI that can drive the same repo; nothing here depends on it.

New to ROS? Start with [tutorial 00](tutorials/00_ubuntu_to_first_topic.md).

## Quick start (host)

First install ROS 2 Jazzy desktop and set up `rosdep` (`sudo rosdep init`, `rosdep update`), see [tutorial 00](tutorials/00_ubuntu_to_first_topic.md) section 2; `scripts/install_ros2.sh` does both. Then clone and install the drivers for the sensors you have:

```bash
git clone https://github.com/cpsl-research/CPSL_ROS2_Sensors
cd CPSL_ROS2_Sensors
bash scripts/install.sh --sensors realsense      # or radar,livox,ouster,realsense,leapmotion,vicon,usb_cam
source /opt/ros/jazzy/setup.bash
source install/setup.bash                        # in every new terminal
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py realsense_enable:=true
```

`install.sh` asks for `sudo`; what it does is in [tutorial 00](tutorials/00_ubuntu_to_first_topic.md). `bash scripts/install_ros2.sh --sensors ...` installs ROS 2 first and then does the same. Flags: `--sensors` (default `radar,livox`), `--ouster-hostname`, `--skip-build`. `--sensors ouster` also sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in your shell rc file (all ROS 2 terminals use it afterwards).

## Quick start (Docker)

The repo has its own image, no host ROS needed ([tutorial 07](tutorials/07_docker.md)):

```bash
bash docker/install_cpsl_sensors_docker.sh --sensors radar,livox
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors
```

`--gpu` and `docker-compose.gpu.yaml` give the CUDA variant; `docker-compose.dev-*.yaml` mount the repo for editing.

## Sensors

`<ns>` is the launch `namespace`; `<camera_name>` is the launch's `camera_name` (`camera` in the default template). Radar names depend on the launch file.

| Sensor | Submodule / package | Topics | Setup |
|---|---|---|---|
| TI radar (serial, DCA1000) | `src/CPSL_TI_Radar` | `<ns>/<radar_name>/detected_points` (point cloud), `adc_data_cube`, `adc_meta` | owned by the radar repo: `bash scripts/install.sh --sensors radar`, then `src/CPSL_TI_Radar/README.md`, `src/CPSL_TI_Radar/docs/tutorials/`, `tools/setup/host_setup.py`; [tutorial 00](tutorials/00_ubuntu_to_first_topic.md) section 7 |
| Livox Mid-360 | `src/CPSL_ROS_livox_ros_driver2` | `<ns>/livox/lidar`, `<ns>/livox/imu` | [03](tutorials/03_livox_mid360_setup.md) |
| Ouster | `src/ouster-ros` | `<ns>/ouster/points`, `imu`, `scan` | [04](tutorials/04_ouster_setup.md) |
| Intel RealSense | `realsense2_camera` (apt, librealsense from source) | `<ns>/camera/<camera_name>/color/image_raw`, `depth/image_rect_raw`, `color/camera_info` | [02](tutorials/02_realsense_setup.md) |
| USB camera | `usb_cam` (apt) | `<ns>/image_raw` | `--sensors usb_cam` |
| Leap Motion | `src/CPSL_ROS2_LeapMotion` | `<ns>/left_hand_joints`, `right_hand_joints`, `leap_image/left`, `leap_image/right` | [05](tutorials/05_leap_motion_setup.md) |
| Vicon | `src/ros2-vicon-bridge` | poses under the `vicon` tf namespace | launch arg `vicon_enable` |

## Launch files

All in `src/cpsl_ros2_sensors_bringup/launch/`; run `ros2 launch cpsl_ros2_sensors_bringup <file> [arg:=value]`. Every sensor has an `*_enable` argument and every file a `namespace`.

| File | Use |
|---|---|
| `default_template_bringup` | all sensors, all off by default; the starting point for a new platform |
| `ugv_sensor_bringup`, `sensor_bringup_single_radar`, `ouster_lidar_bringup` | generic bench bringups |
| `ugv_sensor_bringup_ragnnarok`, `uav_sensor_bringup_radsar`, `uav_sensor_bringup_IcaRAus`, `human_movement_sensor_bringup` | the research setups, [tutorial 08](tutorials/08_paper_setups.md) |

## Recording

```bash
ros2 launch dataset_generator record_dataset.launch.py param_file:=ugv_dataset.yaml dataset_subpath:=run_001
```

One HDF5 file per capture, configured by a YAML in `src/dataset_generator/configs/`. Walkthrough: [tutorial 01](tutorials/01_capturing_a_dataset.md); format: [`hdf5_format.md`](src/dataset_generator/docs/hdf5_format.md); reading it (including transforms and migrating a v1 `frame_N.npy` reader): [`reading_datasets.md`](src/dataset_generator/docs/reading_datasets.md). The recorder needs `python3-h5py` (installed by `install.sh`).

## Tutorials and developer guide

Tutorials 00-08 are in [`tutorials/`](tutorials/): 00 first topic, 01 recording, 02-05 per-sensor setup, [06 adding a platform](tutorials/06_adding_a_platform.md), 07 Docker, 08 paper setups. For packages, conventions, tests and forks see the [developer guide](docs/DEVELOPING.md); open items are in [TODO.md](TODO.md).

# Tutorial 08: The paper setups

Goal: find the launch file, platform description and recording config for each research setup the lab has published data from, so a dataset can be reproduced or extended. These are fixed recipes for specific robots; for your own robot see [Tutorial 06](06_adding_a_platform.md). They need the drivers they start (`bash scripts/install.sh --sensors radar,livox,realsense,leapmotion,vicon`, as needed) and the matching hardware. The radar JSON files named below are in `src/CPSL_TI_Radar/CPSL_TI_Radar_cpp/config/system/`; the radar repo documents the radar side.

---

## 1. The four setups

RaGNNarok, RadSAR/RadVel and IcaRAus are the lab's names for a research project and its robot setup, not software packages; the table says what each one runs. Human movement is the person-tracking rig.

| Setup | Launch file (`cpsl_ros2_sensors_bringup`) | Default namespace | Sensors it can start | Recording configs (`dataset_generator`) |
|---|---|---|---|---|
| RaGNNarok (ground robot) | `ugv_sensor_bringup_ragnnarok.launch.py` | none | Livox, 2 radars, USB camera, `cpsl_ugv_1` description | `ugv_dataset.yaml` |
| RadSAR / RadVel (UAV) | `uav_sensor_bringup_radsar.launch.py` | `cpsl_uav_1` | Livox, front/back/down radars, USB camera | `uav_dataset.yaml`, `uav_dataset_radvel.yaml`, `uav_dataset_radar_vs_flow.yaml` |
| IcaRAus (UAV) | `uav_sensor_bringup_IcaRAus.launch.py` | `cpsl_uav_1` | as above plus Vicon | `uav_dataset_icaraus_dataset_radar.yaml`, `uav_dataset_icaraus_dataset_flow.yaml` |
| Human movement | `human_movement_sensor_bringup.launch.py` | `cpsl_human_movement` | RealSense, Leap Motion (both on by default), radar, Livox | `human_movement.yaml` |

List a launch file's arguments with `ros2 launch cpsl_ros2_sensors_bringup <file> --show-args`. This evaluates the launch file, so it fails with "Could not find the resource" for any driver package that is not installed. `ugv_sensor_bringup.launch.py` and `sensor_bringup_single_radar.launch.py` are the generic bench versions of the ground setup.

## 2. RaGNNarok: ground robot

```bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup_ragnnarok.launch.py
```

Lidar, radar and camera are on by default (`lidar_enable`, `radar_enable`, `camera_enable`; each can be set to `false`). Two radars start: `radar_0` with `IWR1843_demo_RaGNNarok_UGV_5m_front.json` and `radar_1` with `..._back.json`, so their topics are `radar_0/detected_points`, `radar_1/adc_data_cube`, and so on. Livox config is `livox_config_file` ([Tutorial 03](03_livox_mid360_setup.md)). The description is `cpsl_ugv_1.urdf.xml`. `ugv_dataset.yaml` records `radar_0`; copy it and add `radar_1` streams if you use both.

## 3. RadSAR and RadVel: UAV with three radars

```bash
ros2 launch cpsl_ros2_sensors_bringup uav_sensor_bringup_radsar.launch.py \
    front_radar_enable:=true back_radar_enable:=true down_radar_enable:=true lidar_enable:=true
```

Everything except the platform description is off by default. Radar files default to the RadVel 10 Hz set: `IWR1843_demo_RadVel_10Hz_front.json`, `..._back.json` and `IWR6843ODS_demo_RadVel_10Hz_down.json`; override with `front_radar_config_file:=` and the like. The radars are named `front_radar`, `back_radar`, `down_radar`.

The platform description is chosen by the namespace: `namespace:=cpsl_uav_6` loads `cpsl_uav_6.urdf.xml`, and without a namespace `cpsl_uav_1`. `cpsl_uav_6`, `cpsl_uav_8` and `cpsl_uav_9` hold the sensor mounting offsets of the specific drones the datasets were flown on; use the one for your airframe (a `<namespace>.urdf.xml` must exist, so use one of these names or add yours as in Tutorial 06). Recording: `uav_dataset.yaml` (radars, lidar, camera, odometry, IMU), `uav_dataset_radvel.yaml` (radars, lidar, odometry), `uav_dataset_radar_vs_flow.yaml` (the RadVel set plus camera). The UAV presets set `allow_missing: true`: odometry comes from the flight stack, not this repo, and the recorder starts after its timeout even if that topic is silent.

## 4. IcaRAus: UAV with Vicon

Same radar arguments as RadSAR, with IcaRAus radar files (`IWR1843_demo_IcaRAus_front.json`, `..._back.json`, `IWR6843ODS_demo_IcaRAus_10Hz_down.json`) and `vicon_enable:=true`, which starts `vicon_bridge` against a Vicon computer at the address in the launch file (edit the address in the launch file for your lab) and publishes poses under the `vicon` namespace. The two recording configs list the radars and odometry, and a **placeholder** Vicon stream (`vicon/<SUBJECT>/<SEGMENT>`, `enabled: false`). Run the bridge once, read the real pose topic names with `ros2 topic list | grep vicon`, put them in the stream, and set `enabled: true`. The open question about the exact topic layout is tracked in the repo's `TODO.md`.

## 5. Human movement: person tracking

```bash
ros2 launch cpsl_ros2_sensors_bringup human_movement_sensor_bringup.launch.py
```

RealSense (`camera_name:=cpsl_realsense`) and the two Leap Motion nodes start by default, so topics are `camera/cpsl_realsense/color/image_raw`, `left_hand_joints`, `right_hand_joints` ([Tutorial 02](02_realsense_setup.md), [05](05_leap_motion_setup.md)). `radar_enable:=true` adds `radar_0` with `IWR6843ODS_demo_human_movement.json`; `lidar_enable:=true` adds the Livox. RViz (`rviz:=true`) opens `human_view.rviz`. The description is `cpsl_human_movement.urdf.xml`. `human_movement.yaml` records radar, RealSense color and depth and the hand joints. Lidar, IMU, odometry and the two `camera_info` streams are listed but disabled (set `enabled: true` to record them). Enable the sensors you have in the launch, and set `enabled: false` for the others in a copy of the YAML, because the recorder waits for every enabled stream.

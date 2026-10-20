# Tutorial 06: Adding a platform (URDF, bringup launch, recording config)

Goal: describe a new robot's sensor mounting, start its sensors with one launch file, and record it. A platform is three small files that share a name: `platform_descriptions/urdf/<name>.urdf.xml`, `cpsl_ros2_sensors_bringup/launch/<name>_bringup.launch.py`, `dataset_generator/configs/<name>.yaml`. Below the platform is `cpsl_ugv_2`. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md). Every step here runs without sensors.

---

## 1. The URDF: where each sensor sits

A URDF is an XML file listing frames (links) and fixed offsets (joints) between them. Copy the UGV one and rename every frame:

```bash
cd src/platform_descriptions/urdf
cp cpsl_ugv_1.urdf.xml cpsl_ugv_2.urdf.xml
sed -i 's/cpsl_ugv_1/cpsl_ugv_2/g' cpsl_ugv_2.urdf.xml
```

Frame names are `<platform>/<frame>`. The root is `<platform>/base_footprint` (the point on the floor under the robot); each sensor is a link joined to it by a `fixed` joint whose `<origin xyz rpy>` is the sensor's position (metres) and orientation (radians) in the root frame:

```xml
<joint name="radar_0_joint" type="fixed">
    <origin xyz="0.147 0.0 0.11" rpy="0.0 0.0 -1.57079633"/>
    <parent link="cpsl_ugv_2/base_footprint"/>
    <child link="cpsl_ugv_2/radar_0"/>
</joint>
```

Edit the `xyz`/`rpy` of each joint to your measurements and delete sensors you do not have. TI radars report in east-north-up while ROS uses forward-left-up, so radar joints carry a `-1.5708` (-90 degree) yaw like the one above. The Livox joint in the same file has a 3.1416 yaw for its mounting. Sensor links are named as the drivers name their frames (`livox_frame`, `radar_0`, `os_sensor`, `camera_...`), prefixed by the platform name. `default_template.urdf.xml` lists every supported sensor frame with and without the prefix, if you need a longer list.

## 2. The bringup launch file

Copy the generic template and point it at your URDF:

```bash
cd ../../cpsl_ros2_sensors_bringup/launch
cp default_template_bringup.launch.py ugv_2_bringup.launch.py
sed -i "s/default_template.urdf.xml/cpsl_ugv_2.urdf.xml/" ugv_2_bringup.launch.py
```

Also change the `namespace` default from `default_template` to `cpsl_ugv_2`. Read the file once; it has three parts:

- `ARGUMENTS`: the launch arguments. `namespace` prefixes every node and topic; each `*_enable` (`livox_enable`, `front_radar_enable`, `realsense_enable`, ...) switches one sensor on, default `false`.
- `launch_setup()`: reads the arguments, turns the namespace into a `tf_prefix` (`cpsl_ugv_2`) that is passed to the drivers so their frames match the URDF names, and builds one `IncludeLaunchDescription` or `Node` per sensor, each with `condition=IfCondition(<x>_enable)`.
- `generate_launch_description()`: adds the arguments and runs `launch_setup`.

Each sensor sits in a `TimerAction(period=...)`, so they start 3-4 s apart (`default_template_bringup.launch.py:173-326`). Drivers that start together can race for USB or network ports; the stagger prevents it. Keep it when you add or remove sensors, and give a new sensor the next free period. Set radar config files with `front_radar_config_file:=<file>.json` (files from `src/CPSL_TI_Radar/CPSL_TI_Radar_cpp/config/system/`).

Install the two new files, in the terminal where you sourced `/opt/ros/jazzy/setup.bash`:

```bash
cd ../../..
colcon build --packages-select platform_descriptions cpsl_ros2_sensors_bringup --symlink-install
source install/setup.bash
```

## 3. Check the frames

```bash
ros2 launch cpsl_ros2_sensors_bringup ugv_2_bringup.launch.py
```

The description is published 25 s in. In a second terminal:

```bash
ros2 run tf2_ros tf2_echo cpsl_ugv_2/base_footprint cpsl_ugv_2/radar_0
ros2 run tf2_tools view_frames
```

You should see the translation and rotation you wrote in the radar joint (`Translation: [0.147, 0.000, 0.110]`, a -90 degree yaw), and `view_frames` writes a `frames_<time>.pdf` tree with `base_footprint` at the root. `tf2_echo` prints "frame does not exist" until the 25 s have passed.

## 4. The recording config

```bash
cp src/dataset_generator/configs/ugv_dataset.yaml src/dataset_generator/configs/ugv_2_dataset.yaml
```

Edit `dataset_path` and the stream topics (relative, under your namespace), as in [Tutorial 01](01_capturing_a_dataset.md). The v2 recorder has no `base_frame` setting. It always records `/tf_static` (your URDF's mounting transforms) and the pairs listed in `tf_track`, e.g. `["odom->base_link"]`, so a reader can move sensor data into the robot frame later. Rebuild `dataset_generator` once to install the new file, then record with `namespace:=cpsl_ugv_2 param_file:=ugv_2_dataset.yaml`.

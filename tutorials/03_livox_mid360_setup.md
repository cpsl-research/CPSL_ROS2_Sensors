# Tutorial 03: Livox Mid-360 setup

Goal: connect a Livox Mid-360 lidar over Ethernet, point the driver at it, and see the first point cloud. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md). The driver is the lab fork `CPSL_ROS_livox_ros_driver2`, a submodule of this repo.

---

## 1. Install the driver

```bash
bash scripts/install.sh --sensors livox
source install/setup.bash
```

This builds the Livox SDK2 under `/opt/Livox-SDK2` (`scripts/deps/livox.sh`, needs `gcc-9`), checks out `src/CPSL_ROS_livox_ros_driver2`, switches it to the ROS 2 `package.xml` with `build_CPSL_ROS2_Sensors.sh jazzy` (`scripts/install.sh:158`) and builds it.

## 2. Network: the lidar and your PC share 192.168.1.x

The Mid-360 has a fixed address `192.168.1.1XX`, where `XX` are the last two digits of the serial number printed on it. Your PC's wired port needs an address in the same subnet, `192.168.1.YY/24` with any free `YY` (the shipped config uses `192.168.1.78`). Set it in Ubuntu's Settings > Network > the wired connection > IPv4 > Manual, or let the lab's helper do it. With the lidar powered and cabled, discover it (read-only, no root):

```bash
python3 src/CPSL_ROS_livox_ros_driver2/tools/setup/livox_host_setup.py
```

It broadcasts the SDK's search command out of every interface and lists each lidar's serial and IP, even when your PC is on the wrong subnet, and says whether your host can reach it. If it suggests an address, preview and apply it:

```bash
python3 src/CPSL_ROS_livox_ros_driver2/tools/setup/livox_host_setup.py --apply --dry-run
python3 src/CPSL_ROS_livox_ros_driver2/tools/setup/livox_host_setup.py --apply
```

`--apply` only adds a secondary address to the interface the lidar answered on (with `nmcli` if available, otherwise `ip addr add`, which does not survive a reboot). It never replaces existing addresses and calls `sudo` per command, so run it as yourself. `--nic <name>` limits the search to one interface. Check the result with `ping 192.168.1.1XX`.

## 3. Tell the driver the addresses

The driver reads a JSON file in `src/CPSL_ROS_livox_ros_driver2/config/`. The default is `MID360_config.json`. Copy it and edit the copy, so the tracked file stays untouched:

```bash
cd src/CPSL_ROS_livox_ros_driver2/config
cp MID360_config.json MID360_mine.json
```

In `MID360_mine.json` set every `*_ip` under `"host_net_info"` (`cmd_data_ip`, `push_msg_ip`, `point_data_ip`, `imu_data_ip`) to your PC's address, and `"lidar_configs"[0]."ip"` to the lidar's address. Leave the ports. Note that `MID360_config.json` is tracked in the fork but also listed in its `.gitignore`, so edits to it show up as a local change that should not be committed; a copy under another name avoids that. For the Docker image, `MID360_config_docker.json` is the variant to start from ([Tutorial 07](07_docker.md)).

## 4. Launch and see the first cloud

The launch argument `livox_config_file` takes the file name from that folder (`msg_MID360_launch.py:14,33-34`):

```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py \
    livox_enable:=true livox_config_file:=MID360_mine.json
```

The driver starts at once (it is the first entry in `default_template_bringup.launch.py`). In a second terminal:

```bash
ros2 topic hz /default_template/livox/lidar
ros2 topic hz /default_template/livox/imu
```

You should see rates of about 10 Hz for the point cloud (`publish_freq = 10.0` in the fork's launch file) and the IMU rate for `livox/imu`. The cloud is a `sensor_msgs/msg/PointCloud2` in frame `livox_frame`, prefixed with the namespace when `tf_prefix` is set. To look at it, add `rviz:=true` to the launch. If nothing arrives, re-check the host address with `ping`, the file name, and that no other program (including a second driver) is using UDP port 56000-56401.

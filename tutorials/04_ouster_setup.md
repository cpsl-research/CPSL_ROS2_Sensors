# Tutorial 04: Ouster lidar setup

Goal: connect an Ouster OS-series lidar over a direct Ethernet cable, find its address, and see the first scan. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md). The driver is the upstream `ouster-ros`, a submodule of this repo.

---

## 1. Install the driver

```bash
bash scripts/install.sh --sensors ouster
```

This installs the driver's system packages (`scripts/deps/ouster.sh`, including `fping` and the Cyclone DDS middleware), checks out `src/ouster-ros`, and builds `ouster_ros` and `ouster_sensor_msgs` in Release mode before the rest of the workspace. Add `--ouster-hostname os-XXXXXXXXXXXX.local` (or an IP) to also write the sensor name into the driver config (step 3).

It appends `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to your shell's rc file, so every new ROS 2 terminal, not just Ouster ones, uses Cyclone DDS afterwards (remove the line to undo). The Ouster driver needs Cyclone DDS, and every terminal that runs or reads the lidar's topics must use the same setting. Open a new terminal (or `source ~/.bashrc`) and check:

```bash
echo $RMW_IMPLEMENTATION
```

You should see `rmw_cyclonedds_cpp`.

## 2. Network: link-local

Connect the lidar to the PC's wired port, power it, and set that port's IPv4 to manual: address `169.254.1.1`, netmask `255.255.0.0` (Settings > Network > the wired connection > IPv4). The lidar picks its own `169.254.x.y` address and announces the name `os-<serial>.local`, where the serial is on its label. Check that the name resolves:

```bash
ping os-122517001998.local     # replace with your serial
```

If the name does not resolve, scan for the lidar's address instead:

```bash
bash scripts/find_ouster_ip.sh 169.254.1.2 169.254.1.254
```

The script runs `fping` over the range and prints every address that answers. With no arguments it scans the whole `169.254.0.1-169.254.255.255` range, which is slow; narrow it as above and widen it if nothing answers. The address that is not your own is the lidar.

## 3. Tell the driver which lidar

Edit `sensor_hostname` in `src/cpsl_ros2_sensors_bringup/ouster_configs/driver_params.yaml`, keeping the single quotes:

```yaml
    sensor_hostname: 'os-122517001998.local'   # or the IP address found above
```

The installed copy is a link to this file (the workspace is built with `--symlink-install`), so no rebuild is needed. The same file holds the optional `lidar_mode`, `timestamp_mode` and port settings; leaving them empty (or `0` for the ports) lets the driver choose. `timestamp_mode` is set to `TIME_FROM_ROS_TIME` so header stamps use the host clock and line up with the other sensors (the per-point `t` field keeps timing within a scan).

Recording: `configs/ouster.yaml` ([Tutorial 01](01_capturing_a_dataset.md)).

## 4. Launch and see the first scan

```bash
ros2 launch cpsl_ros2_sensors_bringup ouster_lidar_bringup.launch.py
```

In a second terminal (same `RMW_IMPLEMENTATION`, workspace sourced):

```bash
ros2 topic list | grep ouster
ros2 topic hz /ouster/points
```

You should see the driver's topics under `/ouster/` (the launch sets the driver namespace to `ouster`), including `points` (point cloud), `imu` and `scan`, and a rate on `points` matching the lidar's configured mode. Add `rviz:=true` to open a viewer; add `namespace:=foo` to put everything under `/foo/`.

## 5. Troubleshooting

- **The driver exits at start or logs a connection error.** Run `ping` against the hostname or IP in `driver_params.yaml` first; fix the cable and the host address before anything else.
- **Topics exist but stay silent.** The lidar sends UDP to your PC; a firewall (`sudo ufw status`) or a second network interface with another `169.254` address can block or misroute it. The driver config's `udp_dest` can name the destination address explicitly.
- **`ros2 topic list` shows no Ouster topics from another terminal.** That terminal is not on Cyclone DDS; set `RMW_IMPLEMENTATION` as in step 1.
- **A metadata JSON file appears in the folder you launched from.** The driver writes it there when `metadata:` in `driver_params.yaml` is empty; it can be ignored or deleted.
- **In Docker:** use `driver_params_docker.yaml` ([Tutorial 07](07_docker.md)).

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
- PC adapter: `192.168.1.XX` (subnet 255.255.255.0)
- Front DCA1000: `192.168.1.180` (port pair `4096/4098`)
- Back DCA1000: `192.168.1.182` (port pair `4088/4090`)
- See the [DCA1000 Setup instructions](https://github.com/davidmhunt/CPSL_TI_Radar/blob/main/DCA_Programming/README.md)
**Required kernel tuning (apply once, make permanent):**
```bash
sudo sysctl -w net.core.rmem_max=134217728
# permanent:
echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/99-radar.conf
sudo sysctl --system
```

### Livox Mid360

1. Set your PC ethernet adapter to static `192.168.1.XX` (netmask `255.255.255.0`), where `XX` are the last two digits of the sensor's serial number.
2. Update `src/CPSL_ROS_livox_ros_driver2/config/MID360_config.json`:
   ```json
   "host_net_info" : {
      "cmd_data_ip" : "192.168.1.XX",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.XX",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.XX",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.XX",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
   "lidar_configs": { "ip": "192.168.1.1XX", ... }
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
4. **IP Discovery Helper:** If hostname resolution (`os-XXXXXXXXXXXX.local`) is finicky, you can run the discovery helper script to scan local and link-local networks and automatically find the lidar's IP address:
   ```bash
   poetry run python3 scripts/find_ouster_ip.py
   ```
   This script performs a fast, parallelized sweep and queries the Ouster HTTP API to retrieve the exact IP address and sensor details.

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
- [`tutorials/04_docker_setup_and_usage.md`](tutorials/04_docker_setup_and_usage.md) — guide to build and run ROS2 containers
- [`src/CPSL_TI_Radar_ROS2/README.md`](src/CPSL_TI_Radar_ROS2/README.md) — TI radar driver details and config reference
- [`src/CPSL_ROS_livox_ros_driver2/README.md`](src/CPSL_ROS_livox_ros_driver2/README.md) — Livox driver details
- [`src/ouster-ros/README.md`](src/ouster-ros/README.md) — Ouster driver details

---

## Docker Containerization & GUI Support

The application is containerized to support isolated execution on both CPU and GPU architectures. This guarantees identical runtimes, separates ROS2 network discovery from your host machine, and routes GUI displays (like RViz2) seamlessly.

### Prerequisites

#### 1. Docker Engine Setup
Ensure Docker and Docker Compose are installed on your host. If not, follow the [Docker Engine Installation Guide](https://docs.docker.com/engine/install/).

Make sure your user is added to the `docker` group so you can run container commands without `sudo`:
```bash
sudo usermod -aG docker $USER
# Log out and back in for changes to take effect
```

#### 2. NVIDIA Container Toolkit (For GPU Systems)
To run hardware-accelerated GUI interfaces (e.g., RViz2 utilizing your graphics card) or utilize GPU resources inside Docker, install the **NVIDIA Container Toolkit**:
```bash
# Configure the production repository:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit:
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure the runtime:
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```
For detailed setup instructions, visit the [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

#### 3. X11 GUI Display Permissions
For GUI applications running inside the container (like RViz2) to display on your host screen, authorize local connection permissions to the X11 server on the host:
```bash
xhost +local:root
```

---

### Usage

For a detailed step-by-step guide, see the [Docker Setup and Usage Tutorial](tutorials/04_docker_setup_and_usage.md).

#### 1. Building the Docker Images
You can build the containers for either CPU or GPU systems using the wrapper script inside the `docker/` directory. The script will first verify Docker Engine is installed and running:
```bash
# Build the image with all sensors enabled (CPU):
bash docker/install_cpsl_sensors_docker.sh

# Build the image with all sensors enabled (GPU/NVIDIA):
bash docker/install_cpsl_sensors_docker.sh --gpu

# Or customize built sensors (e.g., only build for radar and livox lidar):
bash docker/install_cpsl_sensors_docker.sh --sensors radar,livox
```
The script auto-generates a `.env` file inside the `docker/` directory containing interface mappings, display paths, and chosen sensors for Docker Compose.

#### 2. Running with Docker Compose
Bring up the containers using the respective compose files inside `docker/`:
```bash
# Run CPU-only containers:
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors

# Run GPU hardware-accelerated containers:
docker compose -f docker/docker-compose.gpu.yaml run --rm cpsl_sensors
```

#### 3. Network Isolation
The containers use a custom bridge network `cpsl_net` and run on `ROS_DOMAIN_ID=42`. This ensures that:
- Containers can discover each other and communicate via ROS2.
- ROS2 nodes running inside the container are **completely hidden** from ROS2 nodes running on the host machine (and vice versa), avoiding network naming and discovery interference.

#### 4. Hardware & Peripherals Configuration

When deploying containers in production, physical hardware interfaces must be connected and correctly mapped. If a peripheral device is not plugged in, you must modify the Compose configuration to prevent start errors.

##### TI Radar Serial Passthrough
By default, `docker/docker-compose.cpu.yaml` and `docker/docker-compose.gpu.yaml` expect the TI Radar serial interfaces to be available on the host at `/dev/ttyACM0` and `/dev/ttyACM1`.
- **If connected:** Ensure you are in the `dialout` group on the host.
- **If disconnected (Testing/Development):** Comment out or remove the `devices` block from the compose file, otherwise Docker will throw a device gathering error and refuse to start:
  ```yaml
  # devices:
  #   - "/dev/ttyACM0:/dev/ttyACM0"
  #   - "/dev/ttyACM1:/dev/ttyACM1"
  ```

##### Ethernet Sensors (DCA1000, Livox Mid360, Ouster)
Configure static or link-local IP addresses on the host machine network adapters:
1. **TI Radar DCA1000:** Set host adapter to static IP `192.168.33.30`, netmask `255.255.255.0`.
2. **Livox Mid360 Lidar:** Set host adapter to static IP `192.168.1.78` (or matching custom `.env` IP), netmask `255.255.255.0`.
3. **Ouster Lidar:** Set host adapter to link-local IP `169.254.1.1`, netmask `255.255.0.0`.



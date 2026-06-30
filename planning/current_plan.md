# Current Session Plan

Tasks for Docker Integration & Networking Isolation:

- [x] **C1: Script `install_cpsl_sensors_docker.sh`** — Implement a non-interactive setup script suitable for checking the Docker Engine and running/building containers.
- [x] **C2: Dockerfile Setup** — Design a single parameterized `Dockerfile` for both CPU (`ubuntu:24.04`) and GPU (`nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04`) architectures that installs full ROS2 Jazzy Desktop and clones the repo to a specified commit.
- [x] **C3: Compose Configuration** — Create `docker-compose.cpu.yaml` and `docker-compose.gpu.yaml` configurations featuring custom bridge networks (`ROS_DOMAIN_ID=42`), specific UDP port mappings, serial pass-throughs, and X11 forwarding.

---

## Tasks for Docker Container Feedback (Track F)

- [ ] **F1: Docker Image Tagging & Package Integration** — Add support for custom tagging in `install_cpsl_sensors_docker.sh`. Update `Dockerfile` to copy local workspace and run without specific commit git clone.
- [ ] **F2: Dependency Setup Robustness** — Ensure `apt-get update` runs before `rosdep` installation, configure tolerant quiet mode for `rosdep update`. Add support for `iputils-ping` and `iproute2`.
- [ ] **F3: Shared Volume Mounting** — Update all four CPU/GPU/Simulation docker-compose files to mount the host workspace at `/workspace/CPSL_ROS2_Sensors`.
- [ ] **F4: Lidar IP Discovery Refactor** — Install `fping` and write `find_ouster_ip.sh` to replace `find_ouster_ip.py`.
- [ ] **F5: Rebuild Instructions** — Create `rebuild.sh` script and document workspace rebuilding steps.
- [ ] **F6: Docker Networking Note** — Document host IP set to `0.0.0.0` in container networking configurations.


---

## Tasks for Host Device Mapping, Ouster Networking, and Template Bringup (Track G)

- [x] **G1: Host udev Setup Scripts** — Implement two separate setup scripts under `scripts/`:
   - `scripts/setup_radar_udev.sh`: installs udev rules on the host for TI Radars (XDS110 and CP2105 bridges) with proper group permissions.
   - `scripts/setup_realsense_udev.sh`: installs udev rules on the host for Intel RealSense cameras (matching librealsense rules for video and IMU/HID devices, inlined for offline use).
- [x] **G2: Standalone Device Discovery Script & Role Mapping** — Create a standalone script `scripts/detect_devices.sh` and a JSON configuration file `docker/device_config.json` to scan the host for connected TI Radars and RealSense cameras, matching physical USB paths or serial numbers to specific roles (`FRONT_RADAR`, `BACK_RADAR`, `DOWN_RADAR`, `REALSENSE`) and writing to `.env` (defaulting any unassigned/missing roles to `/dev/null`).
- [x] **G3: Compose Port and Device Passing** — Update the docker compose files (`docker-compose.cpu.yaml`, `docker-compose.gpu.yaml`, `docker-compose.sim-cpu.yaml`, `docker-compose.sim-gpu.yaml`) to map dynamic radar and camera devices (via role-based `.env` variables: `FRONT_RADAR_CLI`, `FRONT_RADAR_DATA`, `BACK_RADAR_CLI`, etc.) and add Ouster Lidar network port mappings (`7502`, `7503` UDP).
- [x] **G4: Simple Default URDF** — Create `src/platform_descriptions/urdf/default_template.urdf.xml` connecting all potential sensor frames (Livox, Ouster, TI Radars, RealSense, Leap Motion, Vicon) to `base_link` at `0,0,0,0`.
- [x] **G5: Default Template Bringup Launch** — Create `src/cpsl_ros2_sensors_bringup/launch/default_template_bringup.launch.py` to bring up all sensors with their parameters, with enables defaulted to `false` and staggered startup delays.
- [x] **G6: Tutorials & Documentation Updates** — Document the dynamic device discovery script, role mapping configuration, Ouster port configuration, and template bringup usage. Write a new, detailed tutorial `tutorials/06_udev_setup.md` specifically explaining how to set up the host udev rules, find VID/PID/interface numbers for different radar boards, and run the udev setup scripts.


---

## Tasks for RealSense, Docker-Specific Configs, and Env Integrity (Track H)

- [x] **H1: USBGuard & RealSense HID/USB3.0 Research & Documentation** — Research usbguard rules for RealSense and add them to `tutorials/06_udev_setup.md`. Relies on host-side device recognition and udev/usbguard authorization, so do NOT mount `/dev/bus/usb` inside containers.
- [x] **H2: RealSense Parameter & Docker Streaming Verification** — Verify full RGB, Depth, and RGB+D streaming and parameter configuration inside Docker container.
- [x] **H3: Non-Destructive Env Configuration** — Modify `install_cpsl_sensors_docker.sh` to update `.env` and `docker/.env` in-place, preserving variables like dynamic device mappings.
- [x] **H4: Docker-Specific Configuration Files for Sensors** — Create Docker-specific JSON configurations:
   - TI Radar system config with `/dev/ti_..._cli` / `/dev/ti_..._data` and DCA1000 host IP as `0.0.0.0`.
   - Livox Lidar config (`MID360_docker_config.json`) with `0.0.0.0` IP binding.
   - Ouster Lidar config (`driver_params_docker.yaml`) with container port setup.
- [x] **H5: Align Dev Compose Devices** — Make sure device mapping arrays in dev compose files are fully aligned with the production compose files (e.g. including all `REALSENSE_DEV_*` and `REALSENSE_HID_*` devices).
- [x] **H6: Install `librealsense` and `realsense-viewer` in Container** — Compile and install the full graphical SDK from source inside the Docker image using the user-space `FORCE_RSUSB_BACKEND` driver.
- [x] **H7: RealSense Viewer Debugging Tutorial** — Create a tutorial/section explaining X11 permission configuration and launching the viewer.

---

## Docker Verification Plan & Commands

To verify the Docker containerization, networking isolation, and GUI support, run the following verification steps.

### A. Testing Without Peripherals (Mock/Simulation Mode)

> [!WARNING]
> **Missing Devices Error:** If you run `docker compose` without the TI Radar serial devices plugged into your host, Docker will fail with an error:
> `Error response from daemon: error gathering device information while adding custom device "/dev/ttyACM0": no such file or directory`.
> 
> **How to run without peripherals:**
> 1. Open [docker-compose.cpu.yaml](file:///home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker-compose.cpu.yaml) and [docker-compose.gpu.yaml](file:///home/david/Documents/cpsl-sensors-project/CPSL_ROS2_Sensors/docker-compose.gpu.yaml).
> 2. Comment out or delete the `devices` block:
>    ```yaml
>    # devices:
>    #   - "/dev/ttyACM0:/dev/ttyACM0"
>    #   - "/dev/ttyACM1:/dev/ttyACM1"
>    ```

#### Step 1: GPU Image Build (C2)
Build the GPU Docker image to verify compilation against the CUDA/NVIDIA environment:
```bash
sg docker -c "bash scripts/install_cpsl_sensors_docker.sh --sensors all --gpu"
```

#### Step 2: Compose Configuration Validation (C3)
Check the syntax of the compose files:
```bash
sg docker -c "docker compose -f docker-compose.cpu.yaml config"
sg docker -c "docker compose -f docker-compose.gpu.yaml config"
```

#### Step 3: Inter-Container ROS2 Discovery & Host Isolation Test (C3)
We verify that ROS2 nodes running inside the docker environment can discover each other, but do not leak/interfere with the host.
1. **Terminal 1 - Launch Listener:**
   ```bash
   sg docker -c "docker compose -f docker-compose.cpu.yaml run --rm cpsl_sensors ros2 run demo_nodes_cpp listener"
   ```
2. **Terminal 2 - Launch Talker:**
   ```bash
   sg docker -c "docker compose -f docker-compose.cpu.yaml run --rm cpsl_sensors ros2 run demo_nodes_cpp talker"
   ```
3. **Verify Host Isolation:**
   While the talker and listener are communicating inside the container environment, run this command on your host terminal:
   ```bash
   ros2 node list
   ```
   *Expected Outcome:* The talker and listener nodes **must not** appear on the host network since they are isolated via `ROS_DOMAIN_ID=42` and the custom docker bridge network `cpsl_net`.

#### Step 4: X11 GUI & RViz2 Forwarding Test (C3)
Verify that GUI applications can render on the host screen:
1. Grant local connection permissions to X11 on your host:
   ```bash
   xhost +local:root
   ```
2. Launch RViz2 inside the container:
   ```bash
   sg docker -c "docker compose -f docker-compose.cpu.yaml run --rm cpsl_sensors rviz2"
   ```
   *Expected Outcome:* The RViz2 GUI window successfully opens and displays on your host monitor.

---

### B. Testing With Peripherals Connected (Production Mode)

Ensure the following physical hardware connections are established on your host machine before uncommenting the `devices` configuration:

1. **TI Radar (Serial Interface):**
   - **Peripherals:** Plug in the TI Radar (e.g., IWR1843/IWR6843) to the host using USB cables.
   - **Verification:** Ensure the ports `/dev/ttyACM0` and `/dev/ttyACM1` (or `/dev/ttyUSB0`/`/dev/ttyUSB1`) exist on the host:
     ```bash
     ls -l /dev/ttyACM*
     ```
   - **Compose Config:** Uncomment the `devices:` block in your `docker-compose.*.yaml` files.

2. **TI Radar DCA1000 (Ethernet Data Streaming):**
   - **Peripherals:** Connect the DCA1000 FPGA ethernet port to the host machine.
   - **Host Setup:** Configure the host ethernet interface with a static IP address:
     - IP Address: `192.168.33.30`
     - Netmask: `255.255.255.0` (24-bit subnet)
     - Gateway: Leave blank or default.

3. **Livox Mid360 Lidar (Ethernet Interface):**
   - **Peripherals:** Connect the Livox Mid360 ethernet interface (via standard switch or injector) to the host machine.
   - **Host Setup:** Configure the host ethernet interface with a static IP address:
     - IP Address: `192.168.1.78` (where `78` is the default host IP, or matching your sensor configuration)
     - Netmask: `255.255.255.0`

4. **Ouster Lidar (Ethernet Interface):**
   - **Peripherals:** Connect the Ouster lidar ethernet port to the host.
   - **Host Setup:** Configure the host interface with a link-local address:
     - IP Address: `169.254.1.1`
     - Netmask: `255.255.0.0` (16-bit subnet)


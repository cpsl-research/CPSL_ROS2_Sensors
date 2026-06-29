# Tutorial 04 - Docker Setup and Usage

This tutorial provides step-by-step instructions for containerizing the `CPSL_ROS2_Sensors` workspace. Docker containerization guarantees identical runtimes, isolates ROS2 networking, and enables GUI/RViz2 forwarding from containers to the host.

---

## 1. Prerequisites

Before building or running the containers, complete the following setup on your host machine:

### A. Docker Engine
Ensure Docker and Docker Compose are installed. If not, follow the [Official Docker Installation Guide](https://docs.docker.com/engine/install/).

Add your user to the `docker` group so you can execute docker commands without `sudo`:
```bash
sudo usermod -aG docker $USER
# Log out and log back in for changes to take effect
```

### B. NVIDIA Container Toolkit (For GPU Acceleration)
To run hardware-accelerated GUI tools (like RViz2) utilizing your graphics card or run GPU nodes, install the **NVIDIA Container Toolkit**:
```bash
# Configure the production repository:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit:
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure the docker runtime and restart the service:
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### C. X11 Display Access
To display GUI applications (such as RViz2) from the container onto your host screen, grant local display connection permissions to the X11 server on the host:
```bash
xhost +local:root
```

### D. Host udev Rules & Device Mappings (For Physical Sensors)
Before deploying the compose stack with physical hardware, you must install custom udev rules on the host. This opens read/write permissions for the TI Radar and RealSense cameras and maps the serial ports to stable symlinks (e.g. `/dev/ti_front_radar_cli`), enabling **non-privileged** container access:
*   Follow [Tutorial 06 - Host udev Rules Setup and Device Mapping](06_udev_setup.md) to discover sensor serials/paths, write `device_config.json`, install host rules, and run the discovery scan before bringing up the container.

---

## 2. Building the Images

All Docker configurations reside in the `docker/` directory. Use the wrapper script to configure and build the images:

```bash
# Build the default CPU image with all sensors enabled:
bash docker/install_cpsl_sensors_docker.sh

# Build the GPU (CUDA-enabled) image:
bash docker/install_cpsl_sensors_docker.sh --gpu

# Customize which sensors are built (e.g., only build for radar and livox lidar):
bash docker/install_cpsl_sensors_docker.sh --sensors radar,livox

# Build with a custom image tag:
bash docker/install_cpsl_sensors_docker.sh --tag custom_name:v1
```

### Script Arguments

| Argument | Description | Default |
|---|---|---|
| `--sensors` | Comma-separated list of sensors to build. Options: `radar`, `livox`, `ouster`, `realsense`, `leapmotion`, `vicon`, `all`. | `all` |
| `--gpu` | Target NVIDIA GPU build using `nvidia/cuda:13.3` base image. | `false` (CPU) |
| `-t`, `--tag` | Custom image tag/name to specify for the generated Docker image. | `cpsl_sensors:cpu` or `:gpu` |
| `--livox-ip` | Last octet of the host Livox IP (patches `.env`). | `78` |
| `--ouster-hostname` | Ouster Lidar hostname or IP (patches `.env`). | `169.254.1.1` |
| `--parent-interface` | The parent physical network interface of your host for IPvlan (e.g. `eth0` or `enp0s31f6`). | `eth0` |
| `--skip-build` | Skip building the image and only write `.env`. | `false` |

The script automatically updates git submodules on the host for selected sensors, and then writes a `.env` file inside the `docker/` directory containing Display paths, parent interface name, sensor choices, and interface IPs for Docker Compose.

---

## 3. Running with Docker Compose

Depending on whether you are running in production/deployment or developing and making live code modifications, choose the appropriate Docker Compose file:

### A. Production / Deployment Mode (Pre-compiled Code)
This mode connects your container directly to the physical network sharing your host's subnet. It assigns a static IP (`192.168.1.42`) to the container and requires a physical gateway router at `192.168.1.1` to route traffic.

Crucially, **Production Mode does not mount the host repository as a volume**. It executes the static, pre-compiled code built directly inside the Docker image layer, ensuring absolute consistency:

```bash
# Start CPU production container:
docker compose -f docker/docker-compose.cpu.yaml up

# Start GPU production container:
docker compose -f docker/docker-compose.gpu.yaml up
```

### B. Development Mode (Live Workspace Mounting)
If you are developing, testing local code changes, or running simulated sensor configurations, use the **Dev** Compose files. This mode mounts your host repository parent directory directly into the container workspace (`/workspace/CPSL_ROS2_Sensors`), so changes are immediately reflected:

```bash
# Start CPU development container:
docker compose -f docker/docker-compose.dev-cpu.yaml up

# Start GPU development container:
docker compose -f docker/docker-compose.dev-gpu.yaml up
```

On launch, the container entrypoint will automatically source ROS2 Jazzy and check for any local workspace builds (`install_docker/setup.bash` or `install/setup.bash`).

---

## 4. Hardware and Peripherals Configuration

### A. TI Radar Serial Passthrough
By default, the compose files expect the TI Radar serial interfaces to be available on the host at `/dev/ttyACM0` and `/dev/ttyACM1`.
- **If connected:** Ensure you are in the `dialout` group on the host (`sudo usermod -aG dialout $USER`).
- **If disconnected (Testing/Development):** Comment out or remove the `devices` block from the compose file to prevent Docker from failing with a device gathering error:
  ```yaml
  # devices:
  #   - "/dev/ttyACM0:/dev/ttyACM0"
  #   - "/dev/ttyACM1:/dev/ttyACM1"
  ```

### B. Shared Workspace Volume Mounting (Development Mode Only)
Development compose files (`docker-compose.dev-cpu.yaml` and `docker-compose.dev-gpu.yaml`) mount the host workspace parent directory as a shared volume at `/workspace/CPSL_ROS2_Sensors` inside the container:
```yaml
    volumes:
      - "../:/workspace/CPSL_ROS2_Sensors"
```
This enables **live code editing**. Any modification you make to source files on your host machine is immediately visible inside the container, and compilation builds (`build_docker/`, `install_docker/`, `log_docker/`) are persisted on your host machine.

In contrast, production compose files (`docker-compose.cpu.yaml` and `docker-compose.gpu.yaml`) do not mount the host workspace, and instead run the compiled codebase embedded inside the Docker image layer directly.

### C. Ethernet Sensors Subnets & IP Discovery
Configure static or link-local IP addresses on your host machine network adapters connected to the physical sensors:
1. **TI Radar DCA1000:** Set host adapter to static IP `192.168.33.30`, netmask `255.255.255.0`.
2. **Livox Mid360 Lidar:** Set host adapter to static IP `192.168.1.78`, netmask `255.255.255.0`.
3. **Ouster Lidar:** Set host adapter to link-local IP `169.254.1.1`, netmask `255.255.0.0`.
   - *Tip:* If Ouster hostname resolution is finicky, use the fping discovery helper script on the host to scan for the lidar IP address:
     ```bash
     bash scripts/find_ouster_ip.sh
     ```
     This script sweeps the link-local IP range using `fping` to locate the sensor.

> [!IMPORTANT]
> **Docker Container Networking and Host IP Binding:**
> When configuring host/system IPs for sensor drivers (such as the TI Radar config JSON files) within the Docker environment, set the host-ip parameter (or system IP) to `0.0.0.0`. This ensures the software inside the container binds to all internal network interfaces and correctly receives the incoming UDP streams forwarded from the host.

---

## 5. Network Isolation

The containers launch inside a custom bridge network `cpsl_net` and run on `ROS_DOMAIN_ID=42`. This ensures that:
- Containers can discover each other and communicate via ROS2.
- ROS2 nodes running inside the container are **completely hidden** from ROS2 nodes running on the host machine (and vice versa), avoiding network naming and discovery interference.
- If you run multiple nodes inside the `docker-compose` environment, they will discover and talk to each other correctly.

---

## 6. Rebuilding the Workspace Packages

Since the host workspace is mounted as a shared volume, you can modify any source code files on your host machine. To compile your changes, you must trigger a colcon build **inside** the container.

We provide a helper script `scripts/rebuild.sh` to handle sourcing and compilation automatically.

### Method A: Execute from the Host (Non-Interactive)
You can trigger a rebuild from your host without entering the container shell:
```bash
docker exec -it cpsl_sensors_dev_cpu bash /workspace/CPSL_ROS2_Sensors/scripts/rebuild.sh
```

### Method B: Execute from Inside the Container
If you are already inside the container shell (`docker exec -it cpsl_sensors_dev_cpu bash`), simply run:
```bash
bash scripts/rebuild.sh
```
Or run the standard colcon command:
```bash
colcon build --symlink-install
```
Remember to source the setup file in any active shells after rebuilding:
```bash
source install/setup.bash
```

---

## 7. Multi-Terminal ROS2 Testing (Talker/Listener Example)

To test ROS2 node discovery and communication within the Docker network environment, you can run multiple commands in separate terminals on your host machine.

### Step 1: Start the Docker Environment
First, ensure that the Docker Compose stack is running (using the dev compose file to mount the workspace). For example, start the CPU dev stack in the background:
```bash
docker compose -f docker/docker-compose.dev-cpu.yaml up -d
```

### Step 2: Open Terminal 1 (The Talker)
On your host machine, open a new terminal window/tab and run the following command to execute a shell inside the running container and start a ROS2 talker node:
```bash
docker exec -it cpsl_sensors_dev_cpu bash -c "ros2 run demo_nodes_cpp talker"
```
*To stop the talker and close this container session, press `Ctrl + C`.*

### Step 3: Open Terminal 2 (The Listener)
Open a second terminal window/tab on your host machine and run the following command to execute a shell inside the container and start a ROS2 listener node:
```bash
docker exec -it cpsl_sensors_dev_cpu bash -c "ros2 run demo_nodes_py listener"
```
*To stop the listener and close this container session, press `Ctrl + C`.*

*(Alternatively, you can just run `docker exec -it cpsl_sensors_dev_cpu bash` first in each terminal and run any ROS2 commands interactively. In this case, press `Ctrl + C` to stop the running node, and type `exit` to close the container session).*

### Step 4: Verify Communication
Observe the listener terminal printing messages received from the talker (e.g., `[INFO] [listener]: I heard: [Hello World: 1]`). This confirms ROS2 discovery and communication is fully operational inside the container network.

### Step 5: Shut Down the Stack
Once testing is complete, stop the container stack:
```bash
docker compose -f docker/docker-compose.dev-cpu.yaml down
```

---

## 7. Hardware Verification Checklist


When launching the environment on the actual deployment machine with physical sensors, use the following checklist to verify complete hardware integration, network routing, and GUI passthrough:

### Step 1: Physical Interface Setup
- **TI Radar Serial USB Connection**: 
  Plug in the TI Radar to a USB port. Check that `/dev/ttyACM0` and `/dev/ttyACM1` are successfully created on the host:
  ```bash
  ls -l /dev/ttyACM*
  ```
- **Livox Mid360 Lidar**: 
  Configure a static IP of `192.168.1.78` (netmask `255.255.255.0`) on the host Ethernet adapter connected to the Livox Lidar.
- **TI Radar DCA1000**: 
  Configure a static IP of `192.168.33.30` (netmask `255.255.255.0`) on the host Ethernet adapter connected to the DCA1000.

### Step 2: IPvlan Network Parent Interface
Determine the name of your host's physical network adapter connected to the router/switch (e.g., `eth0`, `enp0s31f6`, or `wlp0s20f3`):
```bash
ip link show
```

Generate the local environment configuration with the specified parent interface:
```bash
bash docker/install_cpsl_sensors_docker.sh --skip-build --parent-interface <your_parent_interface_name>
```
Verify that the `docker/.env` file has been populated with the correct `HOST_PARENT_INTERFACE` value.

### Step 3: Local Display Permissions
Allow the root user inside Docker to access your local X11 server for visualization:
```bash
xhost +local:root
```

### Step 4: Run the Production Compose Stack
Start the production container in IPvlan mode:
- For **CPU** deployment:
  ```bash
  docker compose -f docker/docker-compose.cpu.yaml up
  ```
- For **GPU** deployment:
  ```bash
  docker compose -f docker/docker-compose.gpu.yaml up
  ```

### Step 5: Verify GUI Forwarding and ROS2 Communication
Open a shell inside the running container (replace `_cpu` with `_gpu` if using the GPU stack):
```bash
docker exec -it cpsl_sensors_cpu bash
```
Run `rviz2` to verify that the GUI window successfully displays on your host screen:
```bash
rviz2
```
Verify that ROS2 nodes are communicating on `ROS_DOMAIN_ID=42` and are hidden from default domain traffic on the host.


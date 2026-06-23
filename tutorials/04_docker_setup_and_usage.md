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
```

### Script Arguments

| Argument | Description | Default |
|---|---|---|
| `--sensors` | Comma-separated list of sensors to build. Options: `radar`, `livox`, `ouster`, `realsense`, `leapmotion`, `vicon`, `all`. | `all` |
| `--gpu` | Target NVIDIA GPU build using `nvidia/cuda:13.3` base image. | `false` (CPU) |
| `--livox-ip` | Last octet of the host Livox IP (patches `.env`). | `78` |
| `--ouster-hostname` | Ouster Lidar hostname or IP (patches `.env`). | `169.254.1.1` |
| `--parent-interface` | The parent physical network interface of your host for IPvlan (e.g. `eth0` or `enp0s31f6`). | `eth0` |
| `--skip-build` | Skip building the image and only write `.env`. | `false` |

The script automatically writes a `.env` file inside the `docker/` directory containing Display paths, parent interface name, sensor choices, and interface IPs for Docker Compose.

---

## 3. Running with Docker Compose

Depending on whether you are deploying in a physical network with real hardware sensors or testing locally in simulation mode, choose the appropriate Docker Compose setup:

### A. Production / Deployment Mode (IPvlan)
This mode connects your container directly to the physical network sharing your host's subnet. It assigns a static IP (`192.168.1.42`) to the container and requires a physical gateway router at `192.168.1.1` to route traffic.

```bash
# Start CPU production container:
docker compose -f docker/docker-compose.cpu.yaml up

# Start GPU production container:
docker compose -f docker/docker-compose.gpu.yaml up
```

### B. Simulation / Mock Mode (Bridge + Gateway Simulator)
If you are doing local testing and do not have a physical router or sensors connected, you can spin up the simulation environment. This creates a virtual Docker bridge network and automatically runs a lightweight `gateway_simulator` container at IP `192.168.1.1` to act as the mock network gateway.

To activate the gateway simulator, pass the `--profile test-router` flag:

```bash
# Start CPU simulation container and gateway simulator:
docker compose -f docker/docker-compose.sim-cpu.yaml --profile test-router up

# Start GPU simulation container and gateway simulator:
docker compose -f docker/docker-compose.sim-gpu.yaml --profile test-router up
```

If you do not want to spin up the gateway simulator and only want to run the isolated sensors container in bridge mode, simply omit the `--profile` flag:
```bash
docker compose -f docker/docker-compose.sim-cpu.yaml up
```

On launch, the entrypoint will automatically source ROS2 Jazzy and compile local workspace builds (`install/setup.bash`).

---

## 4. Hardware and Peripherals Configuration

When deploying containers in production, physical hardware interfaces must be connected and correctly mapped. If a peripheral device is not plugged in, modify the Compose configuration to prevent startup errors.

### A. TI Radar Serial Passthrough
By default, the compose files expect the TI Radar serial interfaces to be available on the host at `/dev/ttyACM0` and `/dev/ttyACM1`.
- **If connected:** Ensure you are in the `dialout` group on the host (`sudo usermod -aG dialout $USER`).
- **If disconnected (Testing/Development):** Comment out or remove the `devices` block from `docker/docker-compose.cpu.yaml` and `docker/docker-compose.gpu.yaml` to prevent Docker from failing with a device gathering error:
  ```yaml
  # devices:
  #   - "/dev/ttyACM0:/dev/ttyACM0"
  #   - "/dev/ttyACM1:/dev/ttyACM1"
  ```

### B. Ethernet Sensors Subnets
Configure static or link-local IP addresses on the host machine network adapters:
1. **TI Radar DCA1000:** Set host adapter to static IP `192.168.33.30`, netmask `255.255.255.0`.
2. **Livox Mid360 Lidar:** Set host adapter to static IP `192.168.1.78`, netmask `255.255.255.0`.
3. **Ouster Lidar:** Set host adapter to link-local IP `169.254.1.1`, netmask `255.255.0.0`.
   - *Tip:* If Ouster hostname resolution is finicky, use the Poetry discovery script on the host to determine the exact IP address:
     ```bash
     poetry run python3 scripts/find_ouster_ip.py
     ```

---

## 5. Network Isolation

The containers launch inside a custom bridge network `cpsl_net` and run on `ROS_DOMAIN_ID=42`. This ensures that:
- Containers can discover each other and communicate via ROS2.
- ROS2 nodes running inside the container are **completely hidden** from ROS2 nodes running on the host machine (and vice versa), avoiding network naming and discovery interference.
- If you run multiple nodes inside the `docker-compose` environment, they will discover and talk to each other correctly.

---

## 6. Hardware Verification Checklist

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
Open a shell inside the running container:
```bash
docker exec -it cpsl_sensors bash
```
Run `rviz2` to verify that the GUI window successfully displays on your host screen:
```bash
rviz2
```
Verify that ROS2 nodes are communicating on `ROS_DOMAIN_ID=42` and are hidden from default domain traffic on the host.


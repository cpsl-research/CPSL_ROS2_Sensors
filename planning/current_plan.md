# Current Session Plan

Tasks for Docker Integration & Networking Isolation:

- [x] **C1: Script `install_cpsl_sensors_docker.sh`** — Implement a non-interactive setup script suitable for checking the Docker Engine and running/building containers.
- [x] **C2: Dockerfile Setup** — Design a single parameterized `Dockerfile` for both CPU (`ubuntu:24.04`) and GPU (`nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04`) architectures that installs full ROS2 Jazzy Desktop and clones the repo to a specified commit.
- [x] **C3: Compose Configuration** — Create `docker-compose.cpu.yaml` and `docker-compose.gpu.yaml` configurations featuring custom bridge networks (`ROS_DOMAIN_ID=42`), specific UDP port mappings, serial pass-throughs, and X11 forwarding.

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


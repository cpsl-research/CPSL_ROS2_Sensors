# Tutorial 07: Running in Docker

Goal: build and run this repo's own Docker image (Ubuntu 24.04, ROS 2 Jazzy, the drivers you choose), on a machine with or without sensors. The containers are not run with `--privileged`: they get named devices, ports and, for the RealSense IMU, a few extra mounts (section 7), and run on their own ROS domain. Nothing else is needed from other repos.

---

## 1. Host prerequisites

- **Docker Engine and the compose plugin:** <https://docs.docker.com/engine/install/ubuntu/>. Then `sudo usermod -aG docker $USER` and log out and in.
- **GPU image only:** the NVIDIA Container Toolkit, <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html>.
- **Display for RViz and the RealSense viewer:** `xhost +local:root` on the host, once per login.

## 2. Build the image

```bash
bash docker/install_cpsl_sensors_docker.sh --sensors radar,livox     # CPU image cpsl_sensors:cpu
bash docker/install_cpsl_sensors_docker.sh --gpu                    # CUDA image cpsl_sensors:gpu, all sensors
```

Options: `--sensors` (a list from `radar,livox,ouster,realsense,leapmotion,vicon`, default `all`; `usb_cam` is not in the image), `--gpu`, `--tag <name>`, `--ouster-hostname <h>`, `--parent-interface <nic>` (default `eth0`, see step 5), `--skip-build` (only write the env files). The script checks Docker is running, checks out the chosen submodules on the host, writes `docker/.env` and `./.env` (sensors, host IPs, display), and runs `docker build`. The image installs each driver's system dependencies with the same `scripts/deps/<driver>.sh` that [Tutorial 00](00_ubuntu_to_first_topic.md) uses and builds the workspace into it. The first build takes a long time.

The development compose file `docker-compose.dev-cpu.yaml` expects the image name `cpsl_sensors_testbench:v0.1-cpu`; for it, add `--tag cpsl_sensors_testbench:v0.1-cpu`.

## 3. Map your devices

Serial radars and RealSense cameras are passed in by device path. `scripts/detect_devices.sh` scans the host and writes the paths into `docker/.env`; the compose files fall back to `/dev/null` for anything absent, so a machine with no hardware still starts. Say which radar is which with a role file (gitignored, per machine):

```bash
cp docker/device_config.example.json docker/device_config.json
bash scripts/detect_devices.sh
```

In `device_config.json`, list each radar as `{"role": "FRONT_RADAR", "id": "<board serial or USB port path>"}` (roles `FRONT_RADAR`, `BACK_RADAR`, `DOWN_RADAR` are the compose variables `FRONT_RADAR_CLI` etc.) and the camera under `realsense`. The scanner prints the serials it finds, and "Found 0 connected ..." when nothing is plugged in. Radar boards appear as `/dev/radar/<id>-cli` and `-data` links once the radar repo's `src/CPSL_TI_Radar/tools/setup/host_setup.py --udev --apply` has run on the host. For the RealSense, run [Tutorial 02](02_realsense_setup.md) step 2 on the host (the rules and `/sys` access are the host's).

## 4. Run

```bash
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors      # GPU: docker-compose.gpu.yaml
```

You get a `bash` prompt with ROS and the workspace already sourced (`docker/docker_entrypoint.sh`); then launch as in the other tutorials. This **production** mode runs the code baked into the image and mounts no host files. For live editing use `docker-compose.dev-cpu.yaml` or `dev-gpu.yaml`, which mount the repo at `/workspace/CPSL_ROS2_Sensors`. After editing, rebuild inside the container:

```bash
bash scripts/rebuild.sh
source install_docker/setup.bash
```

In a container `rebuild.sh` builds into `build_docker/`, `install_docker/`, `log_docker/` (gitignored), never the host's `install/`, and skips drivers not in the image's sensor list.

## 5. Network

The production compose files attach the container to an `ipvlan` network `cpsl_net`, `192.168.1.0/24`, address `192.168.1.42`, on the host interface named by `HOST_PARENT_INTERFACE` (default `eth0`; pass `--parent-interface` to the build script). The Livox and radar addresses in section 6 are on that subnet. The container uses `ROS_DOMAIN_ID=42`, so its nodes are invisible to nodes on the host unless the host also sets `ROS_DOMAIN_ID=42`. The sensors' UDP ports are forwarded to the host addresses `HOST_RADAR_IP` (4096, 4098), `HOST_LIVOX_IP` (56101-56401) and `HOST_OUSTER_IP` (7502, 7503) from `.env`. The dev files publish the same ports on all addresses and use Docker's default network. Check the YAML any time with `docker compose -f docker/docker-compose.cpu.yaml config`.

## 6. Per-sensor settings inside Docker

| Sensor | Host address | Launch argument |
|--------|--------------|-----------------|
| Livox | PC `192.168.1.78` (`HOST_LIVOX_IP`) | `livox_config_file:=MID360_config_docker.json` |
| Ouster | PC `169.254.1.1/16` (`HOST_OUSTER_IP`) | `ouster_config_file:=driver_params_docker.yaml` |
| TI radar + DCA1000 | PC `192.168.33.30/24` (`HOST_RADAR_IP`) | radar JSON with the host IP `0.0.0.0` |

`driver_params_docker.yaml` is `driver_params.yaml` with `lidar_port: 7502` and `imu_port: 7503` instead of `0`, because Docker can forward only fixed ports. Set `sensor_hostname` in it as in [Tutorial 04](04_ouster_setup.md). `MID360_config_docker.json` is the Livox config for this setup ([Tutorial 03](03_livox_mid360_setup.md)). For radars, bind the DCA1000 stream to all interfaces (`"system_ip": "0.0.0.0"`) so packets forwarded from the host arrive.

## 7. RealSense IMU and viewer in the container

The compose files mount `/sys/devices` read-write, set `apparmor=unconfined` and add a device rule, and the entrypoint creates `/dev/iio:device*` nodes at start, because the SDK enables IMU channels by writing under `/sys/devices` and Docker cannot map device names containing a colon. To view the camera, use the kernel-backend build of the tools, `/opt/ros/jazzy/bin/realsense-viewer` and `rs-enumerate-devices`; the source-built ones in `/usr/local/bin` need raw USB access that the container does not have:

```bash
docker compose -f docker/docker-compose.cpu.yaml run --rm cpsl_sensors /opt/ros/jazzy/bin/realsense-viewer
```

## 8. Check the setup

- `bash scripts/test_integration.sh [--host|--docker] [--sensors realsense,radar] [--timeout 20]` builds the workspace, detects attached sensors, starts each driver and passes if its topic appears (e.g. `/camera/camera/color/image_raw`). In a container: `docker compose -f docker/docker-compose.dev-cpu.yaml run --rm cpsl_sensors ./scripts/test_integration.sh`.

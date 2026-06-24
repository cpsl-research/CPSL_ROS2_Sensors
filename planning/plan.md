# CPSL_ROS2_Sensors — Global Project Plan

## Context

This plan covers three major tracks of improvement and integration:

1. **Radar Integration Upgrade** — Update the CPSL_TI_Radar C++ submodule to `phase1-dca1000-fix`, adapt the `ti_radar_connect` ROS2 package to the new directory layout, and rewrite the `CPSL_TI_Radar_ROS2` README. **COMPLETE.**
2. **Documentation & Installation Overhaul** — Rewrite the main README, create a one-shot install script, and write a `tutorials/` directory. **COMPLETE.**
3. **Docker Containerization & Network Isolation** — Containerize the application for CPU and GPU architectures, establish network isolation, support GUI/RViz2, and implement a 2-step socket refactor for bridge networks. **IN PROGRESS.**

---

## Track A: CPSL_TI_Radar Integration Upgrade — COMPLETE ✓

All work on branches `feat/phase1-dca1000-integration` (CPSL_TI_Radar_ROS2) and `feat/ti-radar-phase1-upgrade` (CPSL_ROS2_Sensors).

### Completed work

- **A1** — Pinned `CPSL_TI_Radar` sub-submodule to `phase1-dca1000-fix` branch. Bumped both submodule pointers through to `CPSL_ROS2_Sensors`.
- **A2** — Updated `ti_radar_connect/CMakeLists.txt` to install `config/system/` and `config/radar/` side-by-side (preserving relative paths). Updated `connect_ti_radar_launch.py` to resolve configs from `config/system/`. Fixed stale config reference in `sensor_bringup_single_radar.launch.py`.
- **A3** — Rewrote `CPSL_TI_Radar_ROS2/README.md`: package overview, prerequisites (dialout + rmem_max), build, launch params table, config layout, topics, coordinate frame note.
- **A4 (unplanned)** — Added accessible config directory at `src/ti_radar_connect/config/system/` (11 bringup JSON configs) and `config/radar/nav_configs/` (6 .cfg files). Installed after submodule configs so same-named files here override submodule defaults. Default `config_file` in `connect_ti_radar_launch.py` updated to `front_radar_IWR1843_stress_test.json`.

---

## Track B: Documentation & Installation Overhaul — COMPLETE ✓

### Completed work

- **B1** — Rewrote `README.md`: one-liner description, quick-start with install.sh parameter table, sensor support table, per-sensor hardware setup (serial ports, DCA1000 IP config, Livox IP, Ouster link-local), launch reference table (all 7 bringup files), dataset recording quick-start, links to tutorials.
- **B2** — Wrote `scripts/install.sh`: 10-step installer with `--sensors`, `--livox-ip`, `--ouster-hostname`, `--skip-build` flags. Step 2 selectively inits/deinits git submodules based on `--sensors` so only needed sensor repos are cloned. Per-sensor SDK blocks (gcc-9 + Livox-SDK2, Cyclone DDS, Ultraleap, etc.). `raw_radar_msgs` build guarded behind radar sensor check.
- **B2** — Wrote `scripts/install_ros2.sh`: adds ROS2 Jazzy apt repo, installs `ros-jazzy-desktop` + rosdep, then hands off to `install.sh`.
- **B2** — Fixed `src/CPSL_ROS_livox_ros_driver2/build_CPSL_ROS2_Sensors.sh`: removed the pointless `cp -rf launch_ROS2/ launch/` + `rm -rf launch/` pair. Script now only copies `package_ROS2.xml → package.xml`.
- **B3** — Wrote `tutorials/01_capturing_a_dataset.md`: launch file selection table, annotated dataset YAML, recorder launch args, output folder structure, pre-recording `ros2 topic hz` checklist.
- **B3** — Wrote `tutorials/02_adding_a_new_platform.md`: URDF copy/rename, base_footprint link, sensor joint offsets + mandatory ENU→FLU radar rotation, bringup wiring, TF verification.
- **B3** — Wrote `tutorials/03_writing_a_bringup_launch_file.md`: three-part file structure, TimerAction stagger pattern, IfCondition gating, namespace/tf_prefix, complete annotated minimal example.

**Lower-priority (future)**:
- `tutorials/04_creating_a_radar_profile.md` — `.cfg` + `.json` config creation
- `tutorials/05_vicon_setup.md` — Vicon bridge configuration

---

## Track C: Docker Containerization & GUI Support — COMPLETE ✓

- **C1: Script `install_cpsl_sensors_docker.sh`** — Implement a helper script checking for Docker Engine, configuring parameters, and managing docker builds.
- **C2: Dockerfile Setup** — Design a single parameterized `Dockerfile` using `ARG BASE_IMAGE` (`ubuntu:24.04` or `nvidia/cuda:13.3.0-cudnn-devel-ubuntu24.04`) that installs full ROS2 Jazzy Desktop (with RViz2), and executes the builds.
- **C3: Compose Configuration** — Create `docker-compose.cpu.yaml` and `docker-compose.gpu.yaml` with custom bridge network isolation (`ROS_DOMAIN_ID=42`), specific UDP port mapping configurations, pass-through devices (`devices`), and X11 GUI forwarding support (including hardware-accelerated NVIDIA OpenGL parameters for the GPU Compose file).

---

## Track D: Socket Binding Verification & C++ Refactor — IN PROGRESS

- **D1: JSON Configuration Verification** — Create Docker-specific JSON system configurations mapping `"system_IP"` to `"0.0.0.0"`. Validate that this allows communication on both local hosts and Docker containers.
- **D2: C++ Base Refactor** — Once JSON tests pass, refactor `DCA1000Socket.cpp` to bind listening sockets to `INADDR_ANY` (`0.0.0.0`) by default, maintaining host compatibility while resolving the Docker bridge IP binding issue.

---

## Track E: Documentation & Instructions Update — IN PROGRESS

- **E1: Update README.md** — Add extensive guidelines explaining Docker setup, building containers for CPU vs GPU, Compose bringup, X11 permissions, and sensor network configurations.

---

## Track F: Feedback Improvements from Container Testing — IN PROGRESS

- **F1: Docker Image Tagging & Package Integration** — Add support for custom tagging in `install_cpsl_sensors_docker.sh`. Install workspace packages inside Docker using `COPY` instead of `git clone` from a specific commit.
- **F2: Dependency Setup Robustness** — Ensure `apt-get update` runs before `rosdep init/install` inside the Dockerfile and setup scripts. Make `rosdep update` run in quiet mode and tolerant of key resolution failures. Add support for `iputils-ping` and `iproute2`.
- **F3: Shared Volume Mounting** — Update CPU/GPU/Simulation docker-compose files to mount the host's core ROS package workspace as a shared volume.
- **F4: Lidar IP Discovery Refactor** — Install `fping` and replace the custom Python IP discovery script with a lightweight `find_ouster_ip.sh` bash script.
- **F5: Rebuild Instructions** — Create a `rebuild.sh` script and document workspace rebuilding steps.
- **F6: Docker Networking Note** — Add instructions/notes specifying that the host IP should default to `0.0.0.0` from the container's networking perspective.

---

## Decisions Log

| Date | Decision |
|------|----------|
| 2026-05-18 | Use `phase1-dca1000-fix` branch of CPSL_TI_Radar for relative-path configs and DCA1000 fix |
| 2026-05-18 | Install script assumes ROS2 pre-installed; separate `install_ros2.sh` for convenience |
| 2026-05-18 | Path-portability fix: install both `config/system/` and `config/radar/` to share dir, preserving relative path structure |
| 2026-05-18 | Tutorial priorities: dataset capture, new platform/URDF, new bringup launch file |
| 2026-05-19 | Add accessible config dir at `src/ti_radar_connect/config/` — 11 bringup JSONs + 6 .cfg files, installed after submodule so they override |
| 2026-05-19 | Do NOT call `build_CPSL_ROS2_Sensors.sh` from install script — the dangerous rm -rf lines are commented out but the script still has a pointless launch-dir copy/delete pair. Fixed by removing those lines; script now only copies package_ROS2.xml. |
| 2026-05-19 | Selective submodule init: install.sh inits only submodules for selected sensors, deinits others. README clone command changed from --recurse-submodules to plain git clone. |
| 2026-06-22 | Use separate Compose files (`docker-compose.cpu.yaml` and `docker-compose.gpu.yaml`) with X11 mounts and NVIDIA capabilities. |
| 2026-06-22 | Network isolation achieved via Compose bridge network and `ROS_DOMAIN_ID=42`. |
| 2026-06-22 | Sub-sensory package builds configured via `ARG SENSORS=radar,livox` during Docker build to minimize image size. |
| 2026-06-22 | Two-step verification for socket binding (first custom `0.0.0.0` JSON configs, then refactor C++ code to `INADDR_ANY`). |
| 2026-06-23 | Add custom tagging support for docker images via `--tag` / `-t`. |
| 2026-06-23 | Avoid cloning specific git commit inside Dockerfile; instead copy local workspace and mount core packages as shared volume for live updates. |
| 2026-06-23 | Replace Python ouster discovery script with fping bash script (`find_ouster_ip.sh`). |
| 2026-06-23 | Add rebuild.sh helper script and document rebuilding steps in the docker tutorial. |


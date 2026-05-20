# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Planning Workflow

Two planning files are maintained:

- **`planning/plan.md`** — Global project plan. All phases, architectural decisions, and long-horizon tasks. Update this when decisions are made or phases complete.
- **`planning/current_plan.md`** — Session plan. Contains only the tasks for the current working session. Replace this at the start of each new session with just the tasks being worked on now.

### Process for each session

1. Read `planning/plan.md` to understand project context and phase status.
2. Read `planning/current_plan.md` for the current session's tasks and any user comments.
3. Present a session plan to the user and ask clarifying questions **before implementing**.
4. After user approval, implement tasks one at a time, marking `[x]` as each completes.
5. Update `planning/plan.md` when decisions are finalized or phases complete.

### User feedback convention

The user leaves comments as `> **COMMENT (David):**` blocks directly in the plan files. Read all comments before acting. Do not implement until the plan is approved.

---

## Build Commands

All builds require the Poetry virtual environment to be active first:

```bash
eval $(poetry env activate)
source install/setup.bash  # after building
```

**First-time / full build:**
```bash
# 1. Install ROS deps (skip local raw_radar_msgs)
rosdep install --from-paths src -y --rosdistro=jazzy --skip-keys "raw_radar_msgs"

# 2. Configure Livox driver (first time only)
cd src/CPSL_ROS_livox_ros_driver2 && ./build_CPSL_ROS2_Sensors.sh jazzy && cd ../..

# 3. Build in dependency order
python -m colcon build --packages-select raw_radar_msgs --symlink-install
python -m colcon build --packages-select ouster_ros ouster_sensor_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
python -m colcon build --base-paths src --symlink-install
```

**Incremental build (single package):**
```bash
python -m colcon build --packages-select <package_name> --symlink-install
```

**Run linting tests (flake8/pep257/copyright):**
```bash
python -m colcon test --packages-select <package_name>
python -m colcon test-result --verbose
```

No unit test suite exists for the sensor driver or dataset_generator packages — testing is done by running the hardware in the loop.

## Architecture Overview

### Package Roles

```
cpsl_ros2_sensors_bringup   — launch orchestration only; no nodes, just launch files + configs
dataset_generator            — single Python node that subscribes to all active sensors and
                               writes synchronized frames to disk at a fixed rate
platform_descriptions        — URDF models + robot_state_publisher launch; provides /tf_static
                               for all sensor-to-base transforms
```

The three packages above are written by CPSL. Everything else in `src/` is a third-party submodule:

| Submodule | What it provides |
|-----------|-----------------|
| `CPSL_TI_Radar_ROS2` | `ti_radar_connect_node` — serial/Ethernet driver for TI IWR radars |
| `CPSL_ROS_livox_ros_driver2` | `livox_ros_driver2_node` — Livox Mid360 driver |
| `ouster-ros` | `os_driver_node` — Ouster OS-series LiDAR driver |
| `ros2-vicon-bridge` | `vicon_bridge` — Vicon motion capture → `/tf` |
| `CPSL_ROS2_LeapMotion` | `joint_publisher`, `image_publisher` — Leap Motion hand tracking |

### Configuration Pattern

**Radar profiles** live in:
`src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs/*.json`

Key fields in each JSON: `CLI_port`, `data_port`, `DCA1000_streaming.enabled`, `TI_Radar_config_path`.

**Dataset recording** configs live in `src/dataset_generator/configs/*.yaml`. Each YAML enables/disables sensor streams, sets topic names, `base_frame`, `dataset_path`, and save rates. The `param_file` launch argument is a bare filename (no path) — the launch file resolves it inside the `configs/` directory.

**Ouster driver params** live in `src/cpsl_ros2_sensors_bringup/ouster_configs/driver_params.yaml`.

### Launch File Conventions

All bringup launch files follow the same pattern:
- Accept a `namespace` argument that scopes all node names and topics
- Use `TimerAction` to stagger sensor startup (typically 4 s apart) to prevent initialization races
- Boolean `*_enable` arguments gate each sensor subsystem

Standard `base_frame` naming: `<namespace>/base_footprint` (e.g. `cpsl_ugv_1/base_footprint`).

### TF / Coordinate Frame Notes

- TI Radar outputs use **East-North-Up** convention — a 90° rotation is needed to align with ROS **Forward-Left-Up**. This is handled in the URDF joint definitions, not in the driver.
- All sensor-to-robot transforms are defined in the URDF files under `src/platform_descriptions/urdf/`.

### Adding a New Sensor Configuration

1. Add a radar JSON config in the `CPSL_TI_Radar_cpp/configs/` directory (copy an existing one).
2. Add or edit a dataset YAML in `src/dataset_generator/configs/`.
3. Add or edit a bringup launch file in `src/cpsl_ros2_sensors_bringup/launch/`, following the `TimerAction` stagger pattern.
4. If the platform changes physically, update or add a URDF in `src/platform_descriptions/urdf/` and reference it in the bringup launch.

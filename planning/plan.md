# CPSL_ROS2_Sensors — Global Project Plan

## Context

This plan covers two major improvement tracks:

1. **Radar Integration Upgrade** — Update the CPSL_TI_Radar C++ submodule to `phase1-dca1000-fix`, adapt the `ti_radar_connect` ROS2 package to the new directory layout, and rewrite the `CPSL_TI_Radar_ROS2` README.
2. **Documentation & Installation Overhaul** — Rewrite the main README, create a one-shot install script, and write a `tutorials/` directory.

---

## Track A: CPSL_TI_Radar Integration Upgrade

### Background

The `CPSL_TI_Radar` C++ library (nested submodule inside `CPSL_TI_Radar_ROS2`) has a `phase1-dca1000-fix` branch that:
- Restructures configs: `CPSL_TI_Radar_cpp/configs/` → `CPSL_TI_Radar_cpp/config/system/` (JSON system configs), and `configurations/` → `CPSL_TI_Radar_cpp/config/radar/` (`.cfg` chirp profiles)
- Fixes DCA1000 packet drops with a dedicated RX thread
- Replaces every hard-coded absolute `TI_Radar_config_path` in all 37 JSON files with portable relative paths (`../radar/<subdir>/<file>.cfg`)
- Patches `SystemConfigReader.cpp` to resolve relative paths relative to the JSON file's own directory
- Restructures the repo (archives Python streaming code, rewrites README)

The `ti_radar_connect` ROS2 package currently installs from `CPSL_TI_Radar_cpp/configs/` and resolves config paths under `share/ti_radar_connect/configs/`. These both need updating.

### Phase A1: Pin CPSL_TI_Radar submodule to `phase1-dca1000-fix` [ ]

**Location**: `src/CPSL_TI_Radar_ROS2/` (itself a submodule of the main repo)

Steps:
- [ ] A1.1 — In `CPSL_TI_Radar_ROS2`, update the CPSL_TI_Radar sub-submodule: `git -C src/ti_radar_connect/include/CPSL_TI_Radar checkout phase1-dca1000-fix && git -C src/CPSL_TI_Radar_ROS2 add src/ti_radar_connect/include/CPSL_TI_Radar && git -C src/CPSL_TI_Radar_ROS2 commit`
- [ ] A1.2 — Bump the `CPSL_TI_Radar_ROS2` submodule pointer in the main repo and commit.

### Phase A2: Adapt `ti_radar_connect` to new config layout [ ]

**Files to change**:

1. `src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/CMakeLists.txt`
   - Change the install rule that copies `configs` → change to install `config/system` and `config/radar` side-by-side so that relative paths in the JSON files (`../radar/...`) still resolve correctly after installation.
   - New install rules:
     ```cmake
     install(DIRECTORY
       include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/config/system
       DESTINATION share/${PROJECT_NAME}/config
     )
     install(DIRECTORY
       include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/config/radar
       DESTINATION share/${PROJECT_NAME}/config
     )
     ```

2. `src/CPSL_TI_Radar_ROS2/src/ti_radar_connect/launch/connect_ti_radar_launch.py`
   - Change `config_directory_path = "configs"` → `config_directory_path = os.path.join("config", "system")`

3. Verify all bringup launch files in `src/cpsl_ros2_sensors_bringup/launch/` — they pass bare filenames (e.g. `radar_0_IWR1843_vel_sr.json`) which the `connect_ti_radar_launch.py` resolves. No changes should be needed, but confirm filenames match the new `config/system/` directory contents.

Steps:
- [ ] A2.1 — Update `CMakeLists.txt` install rules as above.
- [ ] A2.2 — Update `connect_ti_radar_launch.py` directory path.
- [ ] A2.3 — Audit all JSON filenames referenced in `cpsl_ros2_sensors_bringup` launch files against the new `config/system/` directory — rename any that changed.
- [ ] A2.4 — Rebuild `ti_radar_connect` and verify launch resolves configs.

### Phase A3: Rewrite `CPSL_TI_Radar_ROS2` README [ ]

The current README is outdated (references old catkin, old directory paths, mentions packages that don't exist yet). Rewrite to cover:
- Package overview (`raw_radar_msgs`, `ti_radar_connect`)
- Prerequisites (just the C++ prereqs link + dialout group)
- Build steps in context of this workspace
- Launch usage (the `connect_ti_radar_launch.py` params table)
- JSON config format overview (new `config/system/` + `config/radar/` layout, relative paths)
- Coordinate frame note (ENE → FLU)

- [ ] A3.1 — Rewrite README.

---

## Track B: Documentation & Installation Overhaul

### Phase B1: Rewrite Main README [ ]

Replace the current README with a cleaner, restructured version:
- Brief project description (what this repo is)
- Quick-start section pointing to the install script
- Condensed prerequisites section (ROS2 link, then refer to install script for everything else)
- Hardware setup (radar serial ports, Livox IP, Ouster network — the things the script can't fully automate)
- Launch command reference table (all bringup launch files + key params)
- Dataset recording quick-start
- Link to `tutorials/` for deeper guides

- [ ] B1.1 — Rewrite `README.md`.

### Phase B2: Create Install Scripts [ ]

**`scripts/install.sh`** — Main installer (assumes ROS2 Jazzy already installed):

Parameters / flags:
- `--sensors` — comma-separated list: `radar,livox,ouster,realsense,leapmotion,vicon` (default: `radar,livox`)
- `--livox-ip` — last two digits of Livox serial number (to configure `MID360_config.json`)
- `--ouster-hostname` — Ouster DNS name or IP (to patch `driver_params.yaml`)
- `--skip-build` — install deps only, no colcon build

Script steps (in order):
1. Check for Poetry; install if missing
2. `poetry config virtualenvs.options.system-site-packages true`
3. `poetry env use /usr/bin/python3.12 && poetry install`
4. `rosdep install ...`
5. Per-sensor blocks (guarded by `--sensors`):
   - Livox: build Livox-SDK2 with gcc-9, run `./build_CPSL_ROS2_Sensors.sh`, optionally patch IP
   - Ouster: apt deps, Cyclone DDS setup, optionally patch hostname
   - RealSense: apt install SDK + ROS packages
   - LeapMotion: Ultraleap apt repo + package, `poetry install --with leapmotion`, build cffi bindings
   - Vicon: apt boost + diagnostic-updater
   - Radar: add user to `dialout` group
6. Full colcon build in dependency order (unless `--skip-build`)
7. Print post-install summary (what was installed, what still needs manual hardware config)

**`scripts/install_ros2.sh`** — Convenience wrapper that installs ROS2 Jazzy on Ubuntu 24.04, then calls `install.sh`.

- [ ] B2.1 — Write `scripts/install.sh`.
- [ ] B2.2 — Write `scripts/install_ros2.sh`.

### Phase B3: Create Tutorials Directory [ ]

Create `tutorials/` at the repo root. Each tutorial is a self-contained Markdown file with step-by-step instructions and example commands.

**Priority tutorials**:

1. **`tutorials/01_capturing_a_dataset.md`** — End-to-end dataset capture
   - Launch sensor bringup (picking the right launch file)
   - Create/edit a `dataset_generator` YAML config
   - Launch `record_dataset.launch.py`
   - Verify the saved output (folder structure, file naming)
   - Tips: using `ros2 topic echo` to confirm sensors are publishing before recording

2. **`tutorials/02_adding_a_new_platform.md`** — Adding a new UGV or UAV
   - Copy an existing URDF as starting point
   - Understand joint/link naming conventions and the `base_footprint` root
   - ENE→FLU rotation for radar joints
   - Wire the new URDF into a bringup launch file
   - Verify transforms with `ros2 run tf2_tools view_frames`

3. **`tutorials/03_writing_a_bringup_launch_file.md`** — Composing a multi-sensor launch file
   - Template structure (declare args, `TimerAction` stagger pattern, `IncludeLaunchDescription`)
   - How to add/remove sensor subsystems
   - Namespace conventions
   - Connecting the platform description

**Lower-priority (future)**:
- `tutorials/04_creating_a_radar_profile.md` — `.cfg` + `.json` config creation
- `tutorials/05_vicon_setup.md` — Vicon bridge configuration

- [ ] B3.1 — Write `tutorials/01_capturing_a_dataset.md`.
- [ ] B3.2 — Write `tutorials/02_adding_a_new_platform.md`.
- [ ] B3.3 — Write `tutorials/03_writing_a_bringup_launch_file.md`.

---

## Decisions Log

| Date | Decision |
|------|----------|
| 2026-05-18 | Use `phase1-dca1000-fix` branch of CPSL_TI_Radar for relative-path configs and DCA1000 fix |
| 2026-05-18 | Install script assumes ROS2 pre-installed; separate `install_ros2.sh` for convenience |
| 2026-05-18 | Path-portability fix: install both `config/system/` and `config/radar/` to share dir, preserving relative path structure |
| 2026-05-18 | Tutorial priorities: dataset capture, new platform/URDF, new bringup launch file |

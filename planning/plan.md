# CPSL_ROS2_Sensors — Global Project Plan

## Context

This plan covers two major improvement tracks:

1. **Radar Integration Upgrade** — Update the CPSL_TI_Radar C++ submodule to `phase1-dca1000-fix`, adapt the `ti_radar_connect` ROS2 package to the new directory layout, and rewrite the `CPSL_TI_Radar_ROS2` README. **COMPLETE.**
2. **Documentation & Installation Overhaul** — Rewrite the main README, create a one-shot install script, and write a `tutorials/` directory. **COMPLETE.**

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

# Current Session Plan

## Session Goal
Implement Track A (CPSL_TI_Radar integration upgrade) — this unblocks Track B since the new config layout must be stable before install scripts and tutorials document it.

---

## Phase A1: Pin CPSL_TI_Radar to `phase1-dca1000-fix`

> **Context**: The CPSL_TI_Radar C++ library (a sub-submodule inside `src/CPSL_TI_Radar_ROS2/`) is currently on `main`. The `phase1-dca1000-fix` branch restructures configs into `config/system/` + `config/radar/`, replaces all 37 hard-coded absolute paths with relative paths, fixes DCA1000 packet drops, and rewrites the README.

- [ ] A1.1 — Update the CPSL_TI_Radar sub-submodule to `phase1-dca1000-fix` and commit that change inside the `CPSL_TI_Radar_ROS2` repo.
- [ ] A1.2 — Bump the `CPSL_TI_Radar_ROS2` submodule pointer in the main `CPSL_ROS2_Sensors` repo and commit.

---

## Phase A2: Adapt `ti_radar_connect` to new config layout

> **Context**: `CMakeLists.txt` currently installs `CPSL_TI_Radar_cpp/configs/` → `share/ti_radar_connect/configs`. After the submodule upgrade this directory no longer exists; it is now `config/system/` (JSON) and `config/radar/` (.cfg). The launch file resolves: `os.path.join(share_dir, "configs", filename)` — this path must change to `config/system`.  
> The relative paths inside the JSON files (`../radar/...`) require both `config/system/` and `config/radar/` to be co-installed under the same parent (`share/ti_radar_connect/config/`) so that relative resolution still works at runtime.

- [ ] A2.1 — Update `ti_radar_connect/CMakeLists.txt`:
  - Remove: `install(DIRECTORY include/CPSL_TI_Radar/CPSL_TI_Radar_cpp/configs DESTINATION share/${PROJECT_NAME})`
  - Add: install `config/system` and `config/radar` both under `share/${PROJECT_NAME}/config`
- [ ] A2.2 — Update `connect_ti_radar_launch.py`: change `config_directory_path = "configs"` → `config_directory_path = os.path.join("config", "system")`
- [ ] A2.3 — Audit all radar config filenames referenced in `cpsl_ros2_sensors_bringup` launch files against the new `config/system/` directory. Confirm no renames were introduced.
- [ ] A2.4 — Rebuild `ti_radar_connect` (`python -m colcon build --packages-select ti_radar_connect raw_radar_msgs --symlink-install`) and confirm `install/share/ti_radar_connect/config/system/` and `config/radar/` are populated.

---

## Phase A3: Rewrite `CPSL_TI_Radar_ROS2` README

> **Context**: Current README mentions `catkin`, references old directory paths (`configs/`), lists unfinished packages, and predates the DCA1000 fix. Rewrite for accuracy and brevity.

- [ ] A3.1 — Rewrite `src/CPSL_TI_Radar_ROS2/README.md` covering: package overview, prerequisites, build steps in this workspace, launch usage with params table, new config layout (`config/system/` + `config/radar/`, relative paths), coordinate frame note.

---

## Notes / Blockers

_(Add comments here during implementation using `> **COMMENT (David):**` blocks)_

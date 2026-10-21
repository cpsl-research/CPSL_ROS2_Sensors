# Developing CPSL_ROS2_Sensors

For people changing this repo. To install and use it, start with the [README](../README.md) and the [tutorials](../tutorials/).

## 1. Package map

The repo is a colcon workspace (`src/`). CPSL writes three packages; everything else in `src/` is a submodule.

| Package | What it is |
|---|---|
| `cpsl_ros2_sensors_bringup` | Launch files only (`launch/`), plus the Ouster driver params (`ouster_configs/`) and RViz layouts (`rviz_cfgs/`). It has no nodes. |
| `platform_descriptions` | One launch file, `publish_platform_description.launch.py` (`robot_state_publisher` + `joint_state_publisher` for a URDF), and the URDFs in `urdf/`. |
| `dataset_generator` | The HDF5 recorder `dataset_recorder` (section 4), the converter `hdf5_to_legacy`, `launch/record_dataset.launch.py`, and the recording configs in `configs/`. |

| Submodule (`src/` unless noted) | Role | Origin |
|---|---|---|
| `CPSL_TI_Radar` | TI radar driver (`ros2/ti_radar_connect`, `ros2/raw_radar_msgs`), radar GUI, firmware | lab repo |
| `CPSL_ROS_livox_ros_driver2` | Livox Mid-360 driver | lab fork of Livox-SDK |
| `CPSL_ROS2_LeapMotion` | Leap Motion nodes | lab fork of OpenHRC-aist/leapmotion_ros2 |
| `ouster-ros`, `ros2-vicon-bridge` | Ouster lidar, Vicon | upstream, unmodified |
| `submodules/leapc-python-bindings` | Ultraleap Python bindings | upstream |

## 2. Launch, parameters and URDF: the convention

Every bringup follows one pattern, described step by step in [Tutorial 06](../tutorials/06_adding_a_platform.md):

- **Launch** (`cpsl_ros2_sensors_bringup/launch/<platform>_bringup.launch.py`): a `namespace` argument; one `<sensor>_enable` argument per sensor; sensors wrapped in `TimerAction`s 3-4 s apart; the namespace is also passed to drivers as `tf_prefix` so frame names match the URDF.
- **Parameters** live beside the thing they configure: radar JSON in `src/CPSL_TI_Radar/CPSL_TI_Radar_cpp/config/system/`, Livox JSON in the fork's `config/`, Ouster in `ouster_configs/driver_params.yaml`, recording in `dataset_generator/configs/`. A launch argument such as `livox_config_file` names a file inside that folder; no argument takes a path.
- **URDF** (`platform_descriptions/urdf/<platform>.urdf.xml`): one frame per sensor, named `<platform>/<frame>`, joined to `<platform>/base_footprint` by fixed joints. `default_template.urdf.xml` has all supported sensor frames; the UAV launches pick `<namespace>.urdf.xml`.
- **Launch files import driver packages at load time** (`get_package_share_directory`), so `--show-args` of a bringup needs the drivers it names installed. `default_template_bringup.launch.py` looks them up only for the sensors you enable, so it runs on any build and is the best smoke test.

Published frames follow the REP-105 names `base_footprint`, `base_link`, `odom`, `map`; TI radars output east-north-up, handled in the URDF joint, not in the driver.

## 3. `scripts/deps/`: system dependencies, one script per driver

`scripts/deps/<driver>.sh` (`radar`, `livox`, `ouster`, `realsense`, `leapmotion`, `vicon`, `usb_cam`, plus `dataset` for the recorder's `python3-h5py`) is the **only** place a driver's system dependencies are listed. The contract, from `scripts/deps/_common.sh:1-12`:

- system dependencies only: apt packages, and SDKs built under `/opt` and installed to `/usr/local`. Nothing in the workspace, the shell rc file, group membership or sensor IPs; `install.sh` owns those;
- run as root with no `sudo` inside (`install.sh` calls `sudo bash scripts/deps/<driver>.sh`; the Dockerfile runs it directly);
- idempotent and quick when already satisfied, so a container can re-run it at every start;
- a non-zero exit means the dependencies are not in place.

New driver: add `scripts/deps/<name>.sh`, add the name to `VALID_SENSORS` (`install.sh:37`), the submodule lists (`install.sh:61-76`), the Dockerfile and `rebuild.sh` lists, and `docker/install_cpsl_sensors_docker.sh:56`. Python dependencies are in `pyproject.toml` and `uv.lock`; `scripts/setup_venv.sh` creates `.venv` with `--system-site-packages` (ROS's Python packages are not on PyPI), and the workspace must be built with it active.

Other tools can drive the same scripts and files. The ones known to: `scripts/detect_devices.py`, `scripts/deps/*`, `scripts/setup_venv.sh`, `ouster_configs/driver_params.yaml`, `docker/device_config*.json`, `dataset_generator/configs/*` and `hdf5_to_legacy`. They also write generated files into this tree, ignored in `.gitignore`: `console_generated_*` launch and URDF files and the Livox `MID360_console.json`. Changing a script's arguments or one of these names can break such a tool, so search for the name first.

## 4. The recorder

`dataset_recorder` subscribes to every enabled entry of the YAML's `streams:` table and writes one HDF5 file. Format, YAML keys, QoS, the start gate and transforms are specified in [`src/dataset_generator/docs/hdf5_format.md`](../src/dataset_generator/docs/hdf5_format.md); do not repeat them elsewhere. Reader examples (numpy/h5py, tf, legacy migration) are in [`reading_datasets.md`](../src/dataset_generator/docs/reading_datasets.md); this package's tests run with `cd src/dataset_generator && PYTHONPATH=. python3 -m pytest test/`, which the root `tests/` does not collect. Modules (`src/dataset_generator/dataset_generator/`): `dataset_recorder.py` (the node), `stream_config.py` (parses `streams:`), `converters.py` (message to array, one per message type), `hdf5_writer.py` (file layout), `start_gate.py`, `radar_cfg.py` (derived radar specs from a `.cfg`), `launch_params.py` (writes the parameter file the launch passes in), `hdf5_to_legacy.py`. To record a new message type, add a converter in `converters.py` with a test in `test/test_hdf5_writer.py`, then document it in `hdf5_format.md` (a column-table row).

## 5. Tests

| What | Command (repo root) |
|---|---|
| Repo tests: RealSense helper, no `.friday` links in markdown | `python3 -m pytest tests -q` |
| Recorder unit tests (no ROS needed; skipped parts need `python3-h5py` or `rclpy`) | `cd src/dataset_generator && PYTHONPATH=. python3 -m pytest test/ -q` |
| Lint and package tests through colcon | `colcon test --packages-select dataset_generator cpsl_ros2_sensors_bringup platform_descriptions`, then `colcon test-result --verbose` |
| Start gate and tf capture against fake publishers (~40 s, needs ROS) | `bash src/dataset_generator/test/smoke_gate_tf.sh` |
| Hardware smoke test (builds, finds sensors, checks topics) | `bash scripts/test_integration.sh` |

`colcon test` reports flake8 and pep257 failures in these packages that predate the current documentation work; judge a change by not adding new ones. The recorder tests need `PYTHONPATH=.` because the package is not installed when run this way.

## 6. Lab forks and pins

The lab repo `CPSL_TI_Radar` and the lab forks `CPSL_ROS_livox_ros_driver2` and `CPSL_ROS2_LeapMotion` are separate repos with their own history; change them there, then bump the pin here in its own commit. Rules:

- **Pin on the fork's default branch** (`main` or `master`). A pin that exists only on a feature branch breaks when the branch is merged or deleted.
- **Tag the fork before removing anything** from it (`git tag v1.0 && git push origin v1.0`), so the removal can be recovered. Some forks have no tag yet; cut one first.
- Document only what the lab changed, at the top of the fork's README ("CPSL changes"); the rest stays the upstream text so upstream merges stay clean.
- Never rewrite or force-push a branch a pin points at.

## 7. Releases

`main` is the current line. `v1.0` (annotated tag) is the last state with the old `.npy` recorder and the poetry environment, kept for recovery. Before a release: run the first four rows of section 5 and `bash scripts/test_integration.sh` on a machine with sensors, then tag `vX.Y` on `main`. Use feature branches and merge to `main` by pull request.

## Appendix A: Other scripts and files (install, Ouster, integration and Docker scripts: see README and tutorials)

| Path | Purpose |
|---|---|
| `scripts/setup_venv.sh` | `.venv` with system site-packages; `--leapmotion` adds the Leap bindings |
| `scripts/rebuild.sh` | `colcon build`; inside Docker builds into `build_docker/` and skips drivers not in the image |
| `scripts/detect_devices.py`, `.sh` | Find radar and RealSense devices, write Docker `.env` device paths |
| `scripts/setup_realsense_udev.sh` | Shortcut for `tools/setup/realsense_host_setup.py --apply` |
| `tools/setup/realsense_host_setup.py` | Check/install RealSense udev rules and IMU access (stdlib only; tested by `tests/test_realsense_host_setup.py`) |
| `docker/` | Dockerfile, four compose files, entrypoint, install script, `device_config.example.json` |

## Appendix B: Known problems

Open items are kept in [`TODO.md`](../TODO.md): camera packages pulled in by `rosdep` on every machine, and the Vicon placeholder.


# TODO

- **`cpsl_ros2_sensors_bringup` and `dataset_generator` depend on usb_cam and realsense2_\*.** At `6696bef`, `src/cpsl_ros2_sensors_bringup/package.xml:16-19` and `src/dataset_generator/package.xml:26-29` both list `<depend>` for `usb_cam`, `realsense2_camera`, `realsense2_camera_msgs` and `realsense2_description`, so `rosdep install --from-paths src` installs both camera stacks on every machine, including ones with neither camera. Workaround in console (`6521bc1`): rosdep `--skip-keys` for camera packages the user did not enable. Fix: move the camera launches and their deps into their own package (`<exec_depend>` would only trim build deps; rosdep still installs it).

- **Vicon topics in the UAV presets are a placeholder.** `src/dataset_generator/configs/uav_dataset_icaraus_dataset_flow.yaml` and `..._radar.yaml` carry a disabled `vicon` stream on `vicon/<SUBJECT>/<SEGMENT>` (TransformStamped). The v1 recorder auto-discovered any `/vicon/` topic; v2 needs fixed names, and the `ros2-vicon-bridge` submodule is not checked out, so the real topic layout is unverified. Fix: run `vicon_bridge` once, read the topic names, put them in both YAMLs and set `enabled: true`.

- **`install.sh --livox-ip` is stale.** It edits `lidar_summary_info.lidar_net_info[0].ip` and `host_net_info.host_ip` (`install.sh:163-181`); the current `MID360_config.json` has neither (host addresses are under `MID360.host_net_info`, lidar address in `lidar_configs[0].ip`). Fix or remove the flag; docker/install_cpsl_sensors_docker.sh only uses the value for `.env`.

- **`src/dataset_generator/dataset_generator/depth_node.py`** is an unreferenced copy of an old recorder (no entry point, not imported). Delete it.

# TODO

- **`cpsl_ros2_sensors_bringup` and `dataset_generator` depend on usb_cam and realsense2_\*.** At `6696bef`, `src/cpsl_ros2_sensors_bringup/package.xml:16-19` and `src/dataset_generator/package.xml:26-29` both list `<depend>` for `usb_cam`, `realsense2_camera`, `realsense2_camera_msgs` and `realsense2_description`, so `rosdep install --from-paths src` installs both camera stacks on every machine, including ones with neither camera. Workaround in console (`6521bc1`): rosdep `--skip-keys` for camera packages the user did not enable. Fix: move the camera launches and their deps into their own package (`<exec_depend>` would only trim build deps; rosdep still installs it).

- **Vicon topics in the UAV presets are a placeholder.** `src/dataset_generator/configs/uav_dataset_icaraus_dataset_flow.yaml` and `..._radar.yaml` carry a disabled `vicon` stream on `vicon/<SUBJECT>/<SEGMENT>` (TransformStamped; the layout is verified from the `vicon_bridge` source). Open: the real subject/segment names. Fix: run `vicon_bridge` once, read them with `ros2 topic list | grep vicon`, put them in both YAMLs and set `enabled: true`.


# Tutorial 00: From a fresh Ubuntu to your first topic

Goal: on a clean Ubuntu 24.04 machine, install ROS 2 and this repo, launch one bringup, and read a topic. No other repo is needed. A RealSense camera is the simplest sensor to start with, but steps 1-5 need no hardware.

---

## 1. Four words you will meet

- **Workspace:** a folder with a `src/` directory of ROS packages. Here it is the repo root.
- **colcon:** the build tool. `colcon build` fills `build/`, `install/` and `log/` next to `src/`.
- **Sourcing:** `source install/setup.bash` tells the current terminal where the built packages are. Repeat it in every new terminal.
- **Namespace:** a prefix on node and topic names (with `namespace:=foo`, a topic `joint_states` becomes `/foo/joint_states`). Bringup launch files take a `namespace` argument.

## 2. Install ROS 2 Jazzy

Follow the official guide, install `ros-jazzy-desktop`, and stop before any workspace steps:
<https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>

Then install the tools this repo's scripts use, and initialise `rosdep` (ROS's dependency installer):

```bash
sudo apt install -y git curl python3-rosdep
sudo rosdep init        # skip if it says the file already exists
rosdep update
```

Convenience instead of the two blocks above: `bash scripts/install_ros2.sh --sensors realsense` (run from the cloned repo, step 3) does the ROS install, `rosdep init` and then the install in step 4 in one go (`scripts/install_ros2.sh:13-40`).

## 3. Get the code

```bash
git clone https://github.com/cpsl-research/CPSL_ROS2_Sensors
cd CPSL_ROS2_Sensors
```

Do not clone with `--recursive`: the sensor drivers are git submodules and the install script fetches only the ones you ask for (`scripts/install.sh:57-98`).

## 4. Install and build for a RealSense

```bash
bash scripts/install.sh --sensors realsense
```

What it does, in order (`scripts/install.sh`): checks out the submodules for the chosen sensors, installs `uv` (a Python package manager) if missing, runs `sudo bash scripts/deps/realsense.sh` (apt packages plus librealsense built from source under `/opt`; expect this to take a while), creates `.venv`, runs `rosdep install`, then `colcon build --symlink-install`. It asks for your `sudo` password. It also installs the HDF5 library the recorder needs (`scripts/deps/dataset.sh`).

`--sensors` takes a comma-separated list from `radar,livox,ouster,realsense,leapmotion,vicon,usb_cam` (`install.sh:37`); the default is `radar,livox`. Rerun with another list to add sensors.

Success looks like `Setup complete.` at the end. Then, in this and every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 5. Launch and look (no hardware needed)

`default_template_bringup.launch.py` starts nothing but the robot description until you switch sensors on with `*_enable:=true` arguments:

```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py
```

Sensors are started on a timer, 3 s apart, so the launch does not race (`default_template_bringup.launch.py:173-326`); the description appears after 25 s. In a second terminal (source both files again):

```bash
ros2 topic list
ros2 node list
ros2 topic echo --once /tf_static
```

You should see `/default_template/robot_description`, `/default_template/joint_states`, `/tf` and `/tf_static`, the nodes `/default_template/robot_state_publisher` and `/default_template/joint_state_publisher`, and one message of fixed transforms (sensor mounting positions) from `/tf_static`. Stop the launch with `Ctrl-C`.

## 6. The RealSense

Set up the camera once (USB 3, udev rules, IMU) with [Tutorial 02](02_realsense_setup.md), then:

```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py realsense_enable:=true
```

The camera node starts 13 s in (`default_template_bringup.launch.py:217`). Find its topics and read one:

```bash
ros2 topic list | grep image_raw
ros2 topic hz /default_template/camera/camera/color/image_raw
```

You should see a color `image_raw` topic and a rate printed once frames arrive. The exact topic path is chosen by the upstream `realsense2_camera` driver from the `camera_name:=camera` this launch passes (`default_template_bringup.launch.py:222`), so trust `ros2 topic list` over the path above.

## 7. Next

- Record these topics to a file: [Tutorial 01](01_capturing_a_dataset.md).
- Other sensors: [03 Livox](03_livox_mid360_setup.md), [04 Ouster](04_ouster_setup.md), [05 Leap Motion](05_leap_motion_setup.md). TI radars have their own repo and docs (`src/CPSL_TI_Radar/README.md` after `--sensors radar`).
- Your own robot: [Tutorial 06](06_adding_a_platform.md). Developers: [`docs/DEVELOPING.md`](../docs/DEVELOPING.md).

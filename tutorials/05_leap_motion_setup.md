# Tutorial 05: Leap Motion setup

Goal: install the Ultraleap hand-tracking SDK and the lab's two Leap Motion nodes, and see hand joints and camera images as ROS topics. Prerequisite: [Tutorial 00](00_ubuntu_to_first_topic.md). The ROS side is the lab fork `CPSL_ROS2_LeapMotion` (package `leap_node`). Only two of its nodes are used here; the rest are upstream demos.

---

## 1. Install

```bash
bash scripts/install.sh --sensors leapmotion
```

This does three Leap-specific things:

1. `sudo bash scripts/deps/leapmotion.sh` adds Ultraleap's apt repository (`repo.ultraleap.com`) and installs the `ultraleap-hand-tracking` package. It fails, rather than skipping, if that server is not reachable.
2. It checks out two submodules: `src/CPSL_ROS2_LeapMotion` and `submodules/leapc-python-bindings`.
3. `bash scripts/setup_venv.sh --leapmotion` installs the Python bindings (`import leap`) into the workspace virtual environment `.venv` ([Tutorial 00](00_ubuntu_to_first_topic.md) section 4) and builds their C bridge.

The nodes `import leap`, so the workspace must be built with that venv active, which `install.sh` does. If you rebuild by hand, activate it first:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python -m colcon build --base-paths src --symlink-install
source install/setup.bash
```

(`scripts/rebuild.sh` does the same and activates the venv for you.)

## 2. Check the tracking service and the device

Plug in the Leap Motion controller. The SDK package runs a tracking service that the nodes talk to; check that it is installed and running:

```bash
systemctl list-units --all | grep -i ultraleap
```

You should see an Ultraleap service listed as `active`. If it is listed but not active, start it with `sudo systemctl start <the unit name shown>`. If nothing is listed, the SDK package did not install: re-run `sudo bash scripts/deps/leapmotion.sh` and read its error.

## 3. Launch

The bringup starts the two lab nodes 16 s after launch:

```bash
ros2 launch cpsl_ros2_sensors_bringup default_template_bringup.launch.py leapmotion_enable:=true
```

To start them without any other launch file:

```bash
ros2 run leap_node joint_publisher
ros2 run leap_node image_publisher
```

## 4. Topics

In a second terminal:

```bash
ros2 topic list | grep -E "hand_joints|leap_image"
ros2 topic echo --once /default_template/left_hand_joints
```

You should see these topics (under the launch's namespace; no namespace with `ros2 run`):

| Node | Topic | Type |
|------|-------|------|
| `joint_publisher` | `left_hand_joints`, `right_hand_joints` | `sensor_msgs/msg/PointCloud2`, one point per joint (`x`, `y`, `z` as float32), `frame_id` `cpsl_human_movement/leap_link` |
| `image_publisher` | `leap_image/left`, `leap_image/right` | `sensor_msgs/msg/Image`, `mono8`, `frame_id` `leap_camera_frame` |

`joint_publisher` sets the tracking mode to desktop (the device lies flat, facing up). Hold a hand above the device to get joints. The frame name `cpsl_human_movement/leap_link` is fixed in the node; the platform description for it is `cpsl_human_movement.urdf.xml` ([Tutorial 08](08_paper_setups.md)).

## 5. Recording

The `human_movement.yaml` recording config has streams for `left_hand_joints` and `right_hand_joints` ([Tutorial 01](01_capturing_a_dataset.md)). It also has the `leap_left` and `leap_right` image streams (`leap_image/left|right`, enabled), so the start gate waits for the image node too; set `enabled: false` on them to record joints only.

## 6. Troubleshooting

- **`ModuleNotFoundError: No module named 'leap'`.** The workspace was built without the venv, or `setup_venv.sh --leapmotion` did not run. Repeat step 1.
- **No images.** The image node asks the service for camera images explicitly (`PolicyFlag.Images`); joints can work while images do not. Recheck step 2 and replug the device.

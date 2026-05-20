# Tutorial 02: Adding a New Platform

This tutorial explains how to define a new robot platform — its physical geometry and sensor mounting positions — and wire it into the bringup and dataset recording systems. No prior reading required.

---

## 1. Copy an Existing URDF as a Starting Point

All platform URDFs live in `src/platform_descriptions/urdf/`. Working directory: the **workspace root** (`CPSL_ROS2_Sensors/`).

The UGV description is the simplest existing example:
```bash
# From workspace root:
cp src/platform_descriptions/urdf/cpsl_ugv_1.urdf.xml \
   src/platform_descriptions/urdf/cpsl_ugv_2.urdf.xml
```

Naming convention: `cpsl_<platform_type>_<number>.urdf.xml`  
(e.g. `cpsl_uav_10.urdf.xml`, `cpsl_ugv_2.urdf.xml`)

Open the new file and replace every occurrence of `cpsl_ugv_1` with your new platform name (e.g. `cpsl_ugv_2`).

---

## 2. The Base Link (`base_footprint`)

The first link in the file is the robot's root frame. For ground robots, this is conventionally named `base_footprint`. For UAVs, `base_link` or `base_footprint` (i.e. base link projected to the ground) is used instead.

From `cpsl_ugv_1.urdf.xml`:
```xml
<link name="cpsl_ugv_1/base_footprint">
    <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <!-- TurtleBot4 cylinder: radius 17.1 cm, height 19.2 cm -->
            <cylinder radius="0.171" length="0.192"/>
        </geometry>
        <material name="white">
            <color rgba="1.0 1.0 1.0 0.5"/>
        </material>
    </visual>
</link>
```

`base_footprint` is the ROS convention for the ground-projected center of the robot — the point on the floor directly below the robot's center of mass. It is the parent of all sensor joints. Update the cylinder dimensions to match your platform's footprint.

The visual geometry is for RViz display only; it has no effect on sensor transforms.

---

## 3. Sensor Joints

Each sensor is defined as a child link attached to `base_footprint` via a fixed joint. The joint's `<origin>` sets the sensor's position (xyz in meters) and orientation (rpy in radians) relative to `base_footprint`.

### Livox Mid360

```xml
<link name="cpsl_ugv_2/livox_frame">
    <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <cylinder radius="0.0385" length="0.081"/>
        </geometry>
    </visual>
</link>

<joint name="lidar_joint" type="fixed">
    <!-- xyz: position of the lidar relative to base_footprint (meters) -->
    <!-- rpy: π rotation about Z to align Livox mounting orientation -->
    <origin xyz="0.0 0.0 0.1365" rpy="0.0 0.0 3.141592653589793"/>
    <parent link="cpsl_ugv_2/base_footprint"/>
    <child link="cpsl_ugv_2/livox_frame"/>
</joint>
```

Measure the lidar's center height above the base link origin and set the `z` component accordingly.

### TI Radar

**Important:** TI radars output data in **East-North-Up (ENU)** convention. ROS uses **Forward-Left-Up (FLU)**. The 90° rotation about Z (`rpy="0 0 -1.5707963"`) corrects this mismatch and is **mandatory** for all forward-facing radars. Rear-facing radars use the opposite rotation (`rpy="0 0 1.5707963"`).

```xml
<link name="cpsl_ugv_2/radar_0">
    <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <!-- thin box representing the radar board -->
            <box size="0.0005 0.02 0.05"/>
        </geometry>
        <material name="red">
            <color rgba="1.0 0.0 0.0 1.0"/>
        </material>
    </visual>
</link>

<!-- Front radar: positive x = forward, -π/2 rotation corrects ENU→FLU -->
<joint name="radar_0_joint" type="fixed">
    <origin xyz="0.147 0.0 0.11" rpy="0.0 0.0 -1.57079633"/>
    <parent link="cpsl_ugv_2/base_footprint"/>
    <child link="cpsl_ugv_2/radar_0"/>
</joint>

<!-- Rear radar: negative x = backward, +π/2 rotation for rear-facing ENU→FLU -->
<joint name="radar_1_joint" type="fixed">
    <origin xyz="-0.147 0.0 0.11" rpy="0.0 0.0 1.57079633"/>
    <parent link="cpsl_ugv_2/base_footprint"/>
    <child link="cpsl_ugv_2/radar_1"/>
</joint>
```

Adjust `xyz` to match the physical mounting position on your platform. The `z` value is the height of the radar above `base_footprint`.

### Ouster LiDAR

Ouster uses standard FLU convention — no rotation correction needed. Mount it the same way as the Livox but adjust dimensions:
```xml
<joint name="ouster_joint" type="fixed">
    <origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/>
    <parent link="cpsl_ugv_2/base_footprint"/>
    <child link="cpsl_ugv_2/os_sensor"/>
</joint>
```

---

## 4. Wire Into Bringup and Dataset Recording

### Bringup launch file

The `publish_platform_description.launch.py` file takes a `urdf_file` argument (bare filename, no path). To use your new URDF, update the `urdf_file` argument in the bringup launch file:

```python
# In your bringup launch file (or pass as a launch argument):
launch_arguments=[
    ('urdf_file', 'cpsl_ugv_2.urdf.xml'),
]
```

If you are creating a new bringup launch file, see `tutorials/03_writing_a_bringup_launch_file.md`.

If you are reusing an existing bringup launch file, pass the argument at the command line:
```bash
ros2 launch cpsl_ros2_sensors_bringup ugv_sensor_bringup.launch.py \
    platform_description_enable:=true
```

(The UGV launch files currently hard-code `cpsl_ugv_1.urdf.xml`; to use a different URDF you will need a custom launch file.)

### Dataset YAML

Set `base_frame` in your dataset YAML to match the base link name in the new URDF:

```yaml
# src/dataset_generator/configs/ugv_2_dataset.yaml
dataset_generator:
  ros__parameters:
    base_frame: "cpsl_ugv_2/base_footprint"
    # ... other settings
```

The `base_frame` is used to look up the sensor-to-robot transforms from the TF tree when saving dataset frames.

---

## 5. Verify Transforms

After launching the bringup with `platform_description_enable:=true`, verify the TF tree is correct:

```bash
# Generate a frame tree PDF (creates frames.pdf in the current directory):
ros2 run tf2_tools view_frames

# Or inspect the tree in the terminal:
ros2 run tf2_ros tf2_echo cpsl_ugv_2/base_footprint cpsl_ugv_2/radar_0
```

A correct frame tree looks like:
```
cpsl_ugv_2/base_footprint
├── cpsl_ugv_2/livox_frame     (static, from URDF)
├── cpsl_ugv_2/radar_0         (static, from URDF)
└── cpsl_ugv_2/radar_1         (static, from URDF)
```

Common problems:
- **Missing transforms**: `platform_description_enable` may be `false`, or the URDF filename is wrong.
- **NaN in transform**: URDF XML syntax error; check for mismatched tags.
- **Wrong sensor orientation in RViz**: The ENU→FLU rotation on a radar joint is missing or incorrect; verify `rpy="0 0 -1.57079633"` on front-facing radar joints.

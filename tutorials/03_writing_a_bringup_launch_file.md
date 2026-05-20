# Tutorial 03: Writing a Bringup Launch File

This tutorial explains how to write a new bringup launch file for a custom sensor configuration. No prior reading required.

---

## 1. Three-Part File Structure

Every bringup launch file in this repo uses the same three-part structure:

```
ARGUMENTS       — list of DeclareLaunchArgument objects (top-level module variable)
launch_setup()  — function that reads args and builds the launch actions
generate_launch_description() — entry point; adds ARGUMENTS + OpaqueFunction
```

Minimal skeleton:

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory

pkg_ti_radar_connect      = get_package_share_directory('ti_radar_connect')
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='my_robot',
                          description='Namespace for all nodes and topics'),
    DeclareLaunchArgument('radar_enable', default_value='true',
                          choices=['true', 'false'],
                          description='Start the TI radar driver'),
    DeclareLaunchArgument('platform_description_enable', default_value='true',
                          choices=['true', 'false'],
                          description='Publish URDF and static TF'),
]

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    radar_enable = LaunchConfiguration('radar_enable')
    platform_description_enable = LaunchConfiguration('platform_description_enable')

    namespace_str = namespace.perform(context)
    tf_prefix = namespace_str.strip('/') if namespace_str else ''

    launch_radar = PathJoinSubstitution(
        [pkg_ti_radar_connect, 'launch', 'connect_ti_radar_launch.py']
    )
    launch_platform = PathJoinSubstitution(
        [pkg_platform_descriptions, 'launch', 'publish_platform_description.launch.py']
    )

    bringup_group = GroupAction([
        PushRosNamespace(namespace),
        # ... actions go here (see sections below)
    ])
    return [bringup_group]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
```

`OpaqueFunction` defers `launch_setup` until launch time, which is required to call `.perform(context)` on `LaunchConfiguration` objects.

---

## 2. The `TimerAction` Stagger Pattern

Sensors share hardware buses and network adapters. Launching them simultaneously causes initialization races that can prevent drivers from connecting. `TimerAction` staggers startup by delaying each driver a fixed number of seconds.

Recommended delays:

| Component | Delay |
|-----------|-------|
| Livox LiDAR | 0 s (immediate) |
| First radar | +4 s |
| Second radar | +7 s |
| Third radar | +10 s |
| Platform description (URDF) | +10–13 s |

Example — two radars staggered:

```python
# Radar 0 — starts 4 s after launch
TimerAction(
    period=4.0,
    actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_radar),
        launch_arguments=[
            ('config_file', 'radar_0_config.json'),
            ('radar_name', 'radar_0'),
            ('tf_prefix', tf_prefix),
            ('stamp_delay_sec', '0.1'),
        ],
        condition=IfCondition(radar_enable),
    )]
),

# Radar 1 — starts 7 s after launch
TimerAction(
    period=7.0,
    actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_radar),
        launch_arguments=[
            ('config_file', 'radar_1_config.json'),
            ('radar_name', 'radar_1'),
            ('tf_prefix', tf_prefix),
            ('stamp_delay_sec', '0.1'),
        ],
        condition=IfCondition(radar_enable),
    )]
),
```

The LiDAR is always started immediately (no `TimerAction`) so it has maximum time to complete its initialization before the radars come online.

---

## 3. Gated Subsystems

Each sensor subsystem is guarded by a boolean launch argument so it can be toggled at the command line without editing the file.

Pattern: wrap the `IncludeLaunchDescription` (or `Node`) in a `condition=IfCondition(...)`:

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

radar_enable = LaunchConfiguration('radar_enable')

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(launch_radar),
    launch_arguments=[
        ('config_file', 'my_radar.json'),
        ('radar_name', 'radar_0'),
        ('tf_prefix', tf_prefix),
        ('stamp_delay_sec', '0.1'),
    ],
    condition=IfCondition(radar_enable),   # only starts if radar_enable:=true
)
```

To pass a config filename as a launch argument (useful for UAV launch files with multiple swappable configs):

```python
# In ARGUMENTS:
DeclareLaunchArgument(
    'radar_config_file',
    default_value='radar_0_default.json',
    description='Radar JSON config file'
),

# In launch_setup:
radar_config_file = LaunchConfiguration('radar_config_file')

# In the IncludeLaunchDescription:
launch_arguments=[('config_file', radar_config_file), ...]
```

---

## 4. Namespace and `tf_prefix`

`PushRosNamespace(namespace)` scopes all node names and topics within the `GroupAction` block. A node whose executable publishes `/radar_0/ti_radar` will instead publish `/<namespace>/radar_0/ti_radar`.

`tf_prefix` is passed separately to sensor nodes as a launch argument. Drivers use it to prefix the TF frame IDs they publish, keeping frames from different robots distinct on a shared TF tree.

```python
namespace_str = namespace.perform(context)
if namespace_str:
    if not namespace_str.startswith('/'):
        namespace_str = '/' + namespace_str
    tf_prefix = namespace_str.strip('/')
else:
    tf_prefix = ''

bringup_group = GroupAction([
    PushRosNamespace(namespace),   # scopes topics

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_radar),
        launch_arguments=[
            ('tf_prefix', tf_prefix),  # scopes TF frame IDs
            ...
        ],
    ),
])
```

With `namespace='cpsl_ugv_2'`:
- Topics: `/cpsl_ugv_2/radar_0/ti_radar`
- TF frames: `cpsl_ugv_2/radar_0` (matching the URDF link name)

---

## 5. Complete Minimal Annotated Example

Single radar + platform description, all patterns above demonstrated:

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory

pkg_ti_radar_connect      = get_package_share_directory('ti_radar_connect')
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')

# ── ARGUMENTS ─────────────────────────────────────────────────────────────────
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='cpsl_ugv_2',
                          description='Namespace for all nodes and topics'),
    DeclareLaunchArgument('radar_enable', default_value='true',
                          choices=['true', 'false'],
                          description='Start the TI radar driver'),
    DeclareLaunchArgument('radar_config_file',
                          default_value='radar_0_IWR1843_vel_sr.json',
                          description='Radar JSON config filename'),
    DeclareLaunchArgument('platform_description_enable', default_value='true',
                          choices=['true', 'false'],
                          description='Publish URDF and static TF'),
]

# ── launch_setup ───────────────────────────────────────────────────────────────
def launch_setup(context, *args, **kwargs):
    namespace                  = LaunchConfiguration('namespace')
    radar_enable               = LaunchConfiguration('radar_enable')
    radar_config_file          = LaunchConfiguration('radar_config_file')
    platform_description_enable = LaunchConfiguration('platform_description_enable')

    # Build tf_prefix from the namespace string
    namespace_str = namespace.perform(context)
    tf_prefix = namespace_str.strip('/') if namespace_str else ''

    launch_radar = PathJoinSubstitution(
        [pkg_ti_radar_connect, 'launch', 'connect_ti_radar_launch.py']
    )
    launch_platform = PathJoinSubstitution(
        [pkg_platform_descriptions, 'launch', 'publish_platform_description.launch.py']
    )

    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        # Radar — starts 4 s after launch to avoid initialization races
        TimerAction(
            period=4.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', radar_config_file),
                    ('radar_name', 'radar_0'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1'),
                ],
                condition=IfCondition(radar_enable),
            )]
        ),

        # Platform description — starts 10 s after launch so TF is available
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_platform),
                launch_arguments=[
                    ('urdf_file', 'cpsl_ugv_2.urdf.xml'),
                ],
                condition=IfCondition(platform_description_enable),
            )]
        ),
    ])

    return [bringup_group]

# ── generate_launch_description ───────────────────────────────────────────────
def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
```

Save this file as `src/cpsl_ros2_sensors_bringup/launch/ugv_2_bringup.launch.py` and rebuild:

```bash
# From workspace root:
python -m colcon build --packages-select cpsl_ros2_sensors_bringup --symlink-install
source install/setup.$(basename $SHELL)

ros2 launch cpsl_ros2_sensors_bringup ugv_2_bringup.launch.py
# Or override config at launch time:
ros2 launch cpsl_ros2_sensors_bringup ugv_2_bringup.launch.py \
    radar_config_file:=radar_0_IWR1843_dca.json \
    radar_enable:=true
```

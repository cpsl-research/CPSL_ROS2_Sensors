from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node

#locating other packages
pkg_livox_ros_driver2 = get_package_share_directory('livox_ros_driver2')
pkg_ti_radar_connect = get_package_share_directory('ti_radar_connect')

#ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('radar_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Launch the ti radars (front and back) lidar'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true','false'],
                          description='Display an RViz window with navigation')
]

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    lidar_enable = LaunchConfiguration('lidar_enable')
    radar_enable = LaunchConfiguration('radar_enable')
    rviz = LaunchConfiguration('rviz')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str

    #locating other launch files
    launch_livox = PathJoinSubstitution(
        [pkg_livox_ros_driver2,'launch_ROS2','msg_MID360_launch.py']
    )

    launch_radar = PathJoinSubstitution(
        [pkg_ti_radar_connect,'launch','connect_ti_radar_launch.py']
    )

    # rviz_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'rviz_cfgs', 'slam.rviz'])

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        # SetRemap('/tf', namespace_str + '/tf'),
        # SetRemap('/tf_static', namespace_str + '/tf_static'),
        # SetRemap('/scan', namespace_str + '/scan'),
        # SetRemap('/map', namespace_str + '/map'),
        # SetRemap('/map_metadata', namespace_str + '/map_metadata'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[],
            condition=IfCondition(lidar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file','radar_0_IWR1843_nav.json'),
                ('namespace','radar_0')
            ],
            condition=IfCondition(radar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file','radar_1_IWR1843_nav.json'),
                ('namespace','radar_1')
            ],
            condition=IfCondition(radar_enable)
        )

        # Launch RViz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     namespace=namespace,
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     condition=IfCondition(rviz)
        # )
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

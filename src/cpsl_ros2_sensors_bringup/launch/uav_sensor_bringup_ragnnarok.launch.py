from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node

# locating other packages
pkg_livox_ros_driver2 = get_package_share_directory('livox_ros_driver2')
pkg_ti_radar_connect = get_package_share_directory('ti_radar_connect')
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')
pkg_cpsl_ros2_sensors_bringup = get_package_share_directory('cpsl_ros2_sensors_bringup')

# ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='cpsl_uav_1',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If lidar is enabled, additionally publish a /LaserScan message'),
    DeclareLaunchArgument('front_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the front radar'),
    DeclareLaunchArgument('front_radar_config_file',
                          default_value='front_radar_IWR1843_RaGNNarok_UAV_50m.json',
                          description='Radar config file'),
    DeclareLaunchArgument('back_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the back radar'),
    DeclareLaunchArgument('back_radar_config_file',
                          default_value='back_radar_IWR1843_dca_RadVel_10Hz.json',
                          description='Radar config file'),
    DeclareLaunchArgument('down_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the down radar'),
    DeclareLaunchArgument('down_radar_config_file',
                          default_value='down_radar_IWR6843_ods_dca_RadVel.json',
                          description='Radar config file'),
    DeclareLaunchArgument('camera_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the cameras'),
    DeclareLaunchArgument('platform_description_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Publish the robot description'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true','false'],
                          description='Display RViz')
]

def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace')
    lidar_enable = LaunchConfiguration('lidar_enable')
    lidar_scan_enable = LaunchConfiguration('lidar_scan_enable')
    camera_enable = LaunchConfiguration('camera_enable')
    front_radar_enable = LaunchConfiguration('front_radar_enable')
    front_radar_config_file = LaunchConfiguration('front_radar_config_file')
    back_radar_enable = LaunchConfiguration('back_radar_enable')
    back_radar_config_file = LaunchConfiguration('back_radar_config_file')
    down_radar_enable = LaunchConfiguration('down_radar_enable')
    down_radar_config_file = LaunchConfiguration('down_radar_config_file')
    platform_description_enable = LaunchConfiguration('platform_description_enable')
    rviz = LaunchConfiguration('rviz')

    namespace_str = namespace.perform(context)
    if namespace_str:
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = f'{tf_prefix}/base_level'
        urdf_name = f'{namespace_str.lstrip("/")}.urdf.xml'
    else:
        tf_prefix = ""
        laser_scan_target_frame = "base_level"
        urdf_name = 'cpsl_uav_1.urdf.xml'

    # locate other launch files
    launch_livox = PathJoinSubstitution([pkg_livox_ros_driver2, 'launch_ROS2', 'msg_MID360_launch.py'])
    launch_radar = PathJoinSubstitution([pkg_ti_radar_connect, 'launch', 'connect_ti_radar_launch.py'])
    launch_platform_description = PathJoinSubstitution([pkg_platform_descriptions, 'launch', 'publish_platform_description.launch.py'])
    rviz_config_file = PathJoinSubstitution([pkg_cpsl_ros2_sensors_bringup, 'rviz_cfgs', 'uav_view.rviz'])

    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        # livox lidar (immediate)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[('tf_prefix', tf_prefix)],
            condition=IfCondition(lidar_enable)
        ),

        # front radar (delayed 2s)
        TimerAction(
            period=4.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', front_radar_config_file),
                    ('radar_name', 'front_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1'),
                ],
                condition=IfCondition(front_radar_enable)
            )]
        ),

        # back radar (delayed 4s)
        TimerAction(
            period=7.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', back_radar_config_file),
                    ('radar_name', 'back_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1'),
                ],
                condition=IfCondition(back_radar_enable)
            )]
        ),

        # down radar (delayed 6s)
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', down_radar_config_file),
                    ('radar_name', 'down_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1'),
                ],
                condition=IfCondition(down_radar_enable)
            )]
        ),

        # platform description (delayed 8s)
        TimerAction(
            period=13.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_platform_description),
                launch_arguments=[('urdf_file', urdf_name)],
                condition=IfCondition(platform_description_enable)
            )]
        ),

        # laserscan conversion node (delayed 10s)
        TimerAction(
            period=16.0,
            actions=[Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan_node',
                output='screen',
                parameters=[
                    {'min_height': -0.2},
                    {'max_height': 0.2},
                    {'angle_min': -3.141592653589793},
                    {'angle_max': 3.141592653589793},
                    {'angle_increment': 0.008},
                    {'queue_size': 10},
                    {'scan_time': 1.0/20.0},
                    {'range_min': 0.5},
                    {'range_max': 50.0},
                    {'target_frame': laser_scan_target_frame},
                    {'transform_tolerance': 0.01},
                    {'use_inf': True},
                ],
                condition=IfCondition(lidar_scan_enable),
                remappings=[
                    ('cloud_in', 'livox/lidar'),
                    ('scan', 'livox/scan')
                ],
            )]
        ),

        # camera (delayed 12s)
        TimerAction(
            period=19.0,
            actions=[Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                output='screen',
                condition=IfCondition(camera_enable)
            )]
        ),

        # rviz (delayed 14s)
        TimerAction(
            period=22.0,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,
                arguments=['-d', rviz_config_file],
                output='screen',
                condition=IfCondition(rviz)
            )]
        )
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

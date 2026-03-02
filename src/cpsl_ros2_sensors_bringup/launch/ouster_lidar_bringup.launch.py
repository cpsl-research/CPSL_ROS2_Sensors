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

#locating other packages
pkg_ouster_ros = get_package_share_directory('ouster_ros')
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')
pkg_cpsl_ros2_sensors_bringup = get_package_share_directory('cpsl_ros2_sensors_bringup')

#ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Launch the ouster lidar'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If lidar is enabled, additionally publish a /LaserScan message on the /ouster/scan topic'),
    DeclareLaunchArgument('platform_description_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='publish the robot description corresponding to the sensor locations'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true','false'],
                          description='Display an RViz window')
]

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    lidar_enable = LaunchConfiguration('lidar_enable')
    lidar_scan_enable = LaunchConfiguration('lidar_scan_enable')
    platform_description_enable = LaunchConfiguration('platform_description_enable')
    rviz = LaunchConfiguration('rviz')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str):
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = '{}/os_sensor'.format(tf_prefix)
    else:
        tf_prefix = ""
        laser_scan_target_frame = "os_sensor"

    #locating other launch files
    launch_ouster = PathJoinSubstitution(
        [pkg_ouster_ros,'launch','driver.launch.py']
    )

    launch_platform_description = PathJoinSubstitution(
        [pkg_platform_descriptions,'launch','publish_platform_description.launch.py']
    )

    rviz_config_file = PathJoinSubstitution([pkg_cpsl_ros2_sensors_bringup, 'rviz_cfgs', 'ugv_view.rviz'])

    ouster_params_file = PathJoinSubstitution(
        [pkg_cpsl_ros2_sensors_bringup, 'ouster_configs', 'driver_params.yaml']
    )

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_ouster),
            launch_arguments=[
                ('viz', 'False'), # Disable ouster's own rviz
                ('ouster_ns', 'ouster'),
                ('params_file', ouster_params_file)
            ],
            condition=IfCondition(lidar_enable)
        ),

        # Platform description (delayed 3.0s)
        TimerAction(
            period=3.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_platform_description),
                launch_arguments=[
                    ('urdf_file','cpsl_ugv_1.urdf.xml'), # using an existing default
                ],
                condition=IfCondition(platform_description_enable)
            )]
        ),
        
        #Launch laserscan topic (delayed 6.0s)
        TimerAction(
            period=6.0,
            actions=[Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan_node',
                output='screen',
                parameters=[
                    {'min_height':-0.1},
                    {'max_height':0.1},
                    {'angle_min':-3.141592653589793},
                    {'angle_max':3.141592653589793},
                    {'angle_increment':0.0174532925}, #pi/180
                    {'queue_size':10},
                    {'scan_time':1.0/20.0},
                    {'range_min':0.25},
                    {'range_max':100.0},
                    {'target_frame':laser_scan_target_frame}, #use lidar's point cloud frame
                    {'transform_tolerance':0.01},
                    {'use_inf':True},
                ],
                condition=IfCondition(lidar_scan_enable),
                remappings=[
                    ('cloud_in', 'ouster/points'),  # Remap input point cloud topic
                    ('scan', 'ouster/scan')  # Remap output laser scan topic
                ],
            )]
        ),
        
        # Launch RViz (delayed 9.0s)
        TimerAction(
            period=9.0,
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,
                arguments=['-d', rviz_config_file],
                output='screen',
                parameters=[],
                condition=IfCondition(rviz)
            )]
        )
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

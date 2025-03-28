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
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')
pkg_cpsl_ros2_sensors_bringup = get_package_share_directory('cpsl_ros2_sensors_bringup')

#ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If lidar is enabled, additionally publish a /LaserScan message on the /scan topic'),
    DeclareLaunchArgument('radar_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='Launch the ti radars (front and back) lidar'),
    DeclareLaunchArgument('platform_description_enable',
                          default_value='true',
                          choices=['true','false'],
                          description='publish the robot description corresponding to the sensor locations'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true','false'],
                          description='Display an RViz window with all odometry displayed')
]

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    lidar_enable = LaunchConfiguration('lidar_enable')
    lidar_scan_enable = LaunchConfiguration('lidar_scan_enable')
    radar_enable = LaunchConfiguration('radar_enable')
    platform_description_enable = LaunchConfiguration('platform_description_enable')
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

    launch_platform_description = PathJoinSubstitution(
        [pkg_platform_descriptions,'launch','publish_platform_description.launch.py']
    )

    rviz_config_file = PathJoinSubstitution([pkg_cpsl_ros2_sensors_bringup, 'rviz_cfgs', 'uav_view.rviz'])

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[],
            condition=IfCondition(lidar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file','radar_0_IWR1843_nav.json'),
                ('frame_id','radar_0'),
                ('stamp_delay_sec','0.1'),
            ],
            condition=IfCondition(radar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file','radar_1_IWR1843_nav.json'),
                ('frame_id','radar_1'),
                ('stamp_delay_sec','0.1'),
            ],
            condition=IfCondition(radar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_platform_description),
            launch_arguments=[
                ('urdf_file','x500.urdf.xml')
            ],
            condition=IfCondition(platform_description_enable)
        ),
        
        #Launch laserscan topic
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[
                {'min_height':-0.1},
                {'max_height':0.25},
                {'angle_min':-3.141592653589793},
                {'angle_max':3.141592653589793},
                {'angle_increment':0.0174532925}, #pi/180
                {'queue_size':10},
                {'scan_time':1.0/20.0},
                {'range_min':0.25},
                {'range_max':5.0},
                {'target_frame':'base_link'}, #use lidar's point cloud frame
                {'transform_tolerance':0.01},
                {'use_inf':True},
            ],
            condition=IfCondition(lidar_scan_enable),
            remappings=[
                ('cloud_in', 'livox/lidar'),  # Remap input point cloud topic
                ('scan', 'livox/scan')  # Remap output laser scan topic
            ],
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[],
            condition=IfCondition(rviz)
        )
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

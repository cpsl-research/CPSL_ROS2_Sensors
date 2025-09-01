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
    DeclareLaunchArgument('namespace', default_value='cpsl_uav_1',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If lidar is enabled, additionally publish a /LaserScan message on the /scan topic'),
    DeclareLaunchArgument('front_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the ti radars (front and back) lidar'),
    DeclareLaunchArgument('front_radar_config_file',
                          default_value='front_radar_IWR1843_dca_RadVel_10Hz.json',
                          description='Radar configuration file in install/ti_radar_connect/share/ti_radar_connect/configs folder'),
    DeclareLaunchArgument('back_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the ti radars (front and back) lidar'),
    DeclareLaunchArgument('back_radar_config_file',
                          default_value='back_radar_IWR1843_dca_RadVel_10Hz.json',
                          description='Radar configuration file in install/ti_radar_connect/share/ti_radar_connect/configs folder'),
    DeclareLaunchArgument('down_radar_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the ti radars (front and back) lidar'),
    DeclareLaunchArgument('down_radar_config_file',
                          default_value='down_radar_IWR6843_ods_dca_RadVel.json',
                          description='Radar configuration file in install/ti_radar_connect/share/ti_radar_connect/configs folder'),
    DeclareLaunchArgument('camera_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='Launch the cameras'),
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
    camera_enable = LaunchConfiguration('camera_enable')
    front_radar_enable = LaunchConfiguration('front_radar_enable')
    front_radar_config_file = LaunchConfiguration('front_radar_config_file')
    back_radar_enable = LaunchConfiguration('back_radar_enable')
    back_radar_config_file = LaunchConfiguration('back_radar_config_file')
    down_radar_enable = LaunchConfiguration('down_radar_enable')
    down_radar_config_file = LaunchConfiguration('down_radar_config_file')
    platform_description_enable = LaunchConfiguration('platform_description_enable')
    rviz = LaunchConfiguration('rviz')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str):
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = '{}/base_footprint'.format(tf_prefix)
        urdf_name = '{}.urdf.xml'.format(namespace_str.lstrip('/'))
    else:
        tf_prefix = ""
        laser_scan_target_frame = "base_footprint"
        urdf_name = 'cpsl_uav_1.urdf.xml'

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

        # SetRemap('/tf', namespace_str + '/tf'),
        # SetRemap('/tf_static', namespace_str + '/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[
                ('tf_prefix',tf_prefix)
            ],
            condition=IfCondition(lidar_enable)
        ),

        #start front radar sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file',front_radar_config_file),
                ('radar_name','front_radar'),
                ('tf_prefix',tf_prefix),
                ('stamp_delay_sec','0.1'),
            ],
            condition=IfCondition(front_radar_enable)
        ),

        #start front radar sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file',back_radar_config_file),
                ('radar_name','back_radar'),
                ('tf_prefix',tf_prefix),
                ('stamp_delay_sec','0.1'),
            ],
            condition=IfCondition(back_radar_enable)
        ),

        #start front radar sensor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_radar),
            launch_arguments=[
                ('config_file',down_radar_config_file),
                ('radar_name','down_radar'),
                ('tf_prefix',tf_prefix),
                ('stamp_delay_sec','0.1'),
            ],
            condition=IfCondition(down_radar_enable)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_platform_description),
            launch_arguments=[
                ('urdf_file',urdf_name),
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
                # {'min_height':-0.1},
                # {'max_height':0.1},
                {'min_height':0.5},
                {'max_height':0.90},
                {'angle_min':-3.141592653589793},
                {'angle_max':3.141592653589793},
                {'angle_increment':0.008}, #pi/180
                {'queue_size':10},
                {'scan_time':1.0/20.0},
                {'range_min':0.5},
                {'range_max':50.0},
                {'target_frame':laser_scan_target_frame}, #use lidar's point cloud frame
                {'transform_tolerance':0.01},
                {'use_inf':True},
            ],
            condition=IfCondition(lidar_scan_enable),
            remappings=[
                ('cloud_in', 'livox/lidar'),  # Remap input point cloud topic
                ('scan', 'livox/scan')  # Remap output laser scan topic
            ],
        ),
        
        #start the camera
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output='screen',
            condition=IfCondition(camera_enable)
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

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

# ROS2 launch arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='default_template',
                          description='namespace for the entire platform'),
    DeclareLaunchArgument('livox_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('ouster_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the ouster lidar'),
    DeclareLaunchArgument('front_radar_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the front radar'),
    DeclareLaunchArgument('front_radar_config_file',
                          default_value='front_radar_IWR1843_stress_test.json',
                          description='Configuration file for front radar'),
    DeclareLaunchArgument('back_radar_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the back radar'),
    DeclareLaunchArgument('back_radar_config_file',
                          default_value='front_radar_IWR1843_stress_test.json', # fallback default config
                          description='Configuration file for back radar'),
    DeclareLaunchArgument('down_radar_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the down radar'),
    DeclareLaunchArgument('down_radar_config_file',
                          default_value='front_radar_IWR1843_stress_test.json', # fallback default config
                          description='Configuration file for down radar'),
    DeclareLaunchArgument('realsense_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the Intel RealSense camera'),
    DeclareLaunchArgument('leapmotion_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the Leap Motion hand tracking node'),
    DeclareLaunchArgument('vicon_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the Vicon bridge node'),
    DeclareLaunchArgument('camera_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch the usb webcam node'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Publish LaserScan message from lidar'),
    DeclareLaunchArgument('platform_description_enable',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Publish the default template URDF robot description'),
    DeclareLaunchArgument('livox_config_file',
                          default_value='MID360_config.json',
                          description='Configuration file for Livox Lidar'),
    DeclareLaunchArgument('ouster_config_file',
                          default_value='driver_params.yaml',
                          description='Configuration file for Ouster Lidar'),
    DeclareLaunchArgument('rviz',
                          default_value='false',
                          choices=['true', 'false'],
                          description='Launch RViz2 configured for default template')
]

def launch_setup(context, *args, **kwargs):
    # Load parameters
    namespace = LaunchConfiguration('namespace')
    livox_enable = LaunchConfiguration('livox_enable')
    livox_config_file = LaunchConfiguration('livox_config_file')
    ouster_enable = LaunchConfiguration('ouster_enable')
    ouster_config_file = LaunchConfiguration('ouster_config_file')
    front_radar_enable = LaunchConfiguration('front_radar_enable')
    front_radar_config_file = LaunchConfiguration('front_radar_config_file')
    back_radar_enable = LaunchConfiguration('back_radar_enable')
    back_radar_config_file = LaunchConfiguration('back_radar_config_file')
    down_radar_enable = LaunchConfiguration('down_radar_enable')
    down_radar_config_file = LaunchConfiguration('down_radar_config_file')
    realsense_enable = LaunchConfiguration('realsense_enable')
    leapmotion_enable = LaunchConfiguration('leapmotion_enable')
    vicon_enable = LaunchConfiguration('vicon_enable')
    camera_enable = LaunchConfiguration('camera_enable')
    lidar_scan_enable = LaunchConfiguration('lidar_scan_enable')
    platform_description_enable = LaunchConfiguration('platform_description_enable')
    rviz = LaunchConfiguration('rviz')

    # Resolve tf prefix
    namespace_str = namespace.perform(context)
    if namespace_str:
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = f'{tf_prefix}/base_link'
    else:
        tf_prefix = ""
        laser_scan_target_frame = "base_link"

    # Always required packages
    pkg_platform_descriptions = get_package_share_directory('platform_descriptions')
    pkg_cpsl_ros2_sensors_bringup = get_package_share_directory('cpsl_ros2_sensors_bringup')

    # Conditionally resolved packages to avoid PackageNotFoundError
    pkg_livox_ros_driver2 = get_package_share_directory('livox_ros_driver2') if livox_enable.perform(context) == 'true' else ""
    pkg_ouster_ros = get_package_share_directory('ouster_ros') if ouster_enable.perform(context) == 'true' else ""
    
    radar_any_enable = (front_radar_enable.perform(context) == 'true' or 
                        back_radar_enable.perform(context) == 'true' or 
                        down_radar_enable.perform(context) == 'true')
    pkg_ti_radar_connect = get_package_share_directory('ti_radar_connect') if radar_any_enable else ""
    pkg_realsense2_camera = get_package_share_directory('realsense2_camera') if realsense_enable.perform(context) == 'true' else ""

    # Locate resource and launch files
    launch_livox = PathJoinSubstitution([pkg_livox_ros_driver2, 'launch_ROS2', 'msg_MID360_launch.py']) if pkg_livox_ros_driver2 else ""
    launch_ouster = PathJoinSubstitution([pkg_ouster_ros, 'launch', 'driver.launch.py']) if pkg_ouster_ros else ""
    launch_realsense = PathJoinSubstitution([pkg_realsense2_camera, 'launch', 'rs_launch.py']) if pkg_realsense2_camera else ""
    launch_radar = PathJoinSubstitution([pkg_ti_radar_connect, 'launch', 'connect_ti_radar_launch.py']) if pkg_ti_radar_connect else ""
    launch_platform_description = PathJoinSubstitution([pkg_platform_descriptions, 'launch', 'publish_platform_description.launch.py'])
    
    ouster_params_file = PathJoinSubstitution([pkg_cpsl_ros2_sensors_bringup, 'ouster_configs', ouster_config_file])
    rviz_config_file = PathJoinSubstitution([pkg_cpsl_ros2_sensors_bringup, 'rviz_cfgs', 'default_template.rviz'])

    # Vicon config
    vicon_computer_ip = "192.168.0.101"
    vicon_port = "801"
    vicon_host = f"{vicon_computer_ip}:{vicon_port}"

    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        # ── 1. LIVOX LIDAR (Immediate) ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[
                ('tf_prefix', tf_prefix),
                ('user_config_file', livox_config_file)
            ],
            condition=IfCondition(livox_enable)
        ),

        # ── 2. OUSTER LIDAR (Immediate) ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_ouster),
            launch_arguments=[
                ('viz', 'False'),
                ('ouster_ns', 'ouster'),
                ('params_file', ouster_params_file)
            ],
            condition=IfCondition(ouster_enable)
        ),

        # ── 3. FRONT TI RADAR (Delayed 4.0s) ──────────────────────────────────
        TimerAction(
            period=4.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', front_radar_config_file),
                    ('radar_name', 'front_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1')
                ],
                condition=IfCondition(front_radar_enable)
            )]
        ),

        # ── 4. BACK TI RADAR (Delayed 7.0s) ───────────────────────────────────
        TimerAction(
            period=7.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', back_radar_config_file),
                    ('radar_name', 'back_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.1')
                ],
                condition=IfCondition(back_radar_enable)
            )]
        ),

        # ── 5. DOWN TI RADAR (Delayed 10.0s) ──────────────────────────────────
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_radar),
                launch_arguments=[
                    ('config_file', down_radar_config_file),
                    ('radar_name', 'down_radar'),
                    ('tf_prefix', tf_prefix),
                    ('stamp_delay_sec', '0.0')
                ],
                condition=IfCondition(down_radar_enable)
            )]
        ),

        # ── 6. INTEL REALSENSE CAMERA (Delayed 13.0s) ──────────────────────────
        TimerAction(
            period=13.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_realsense),
                launch_arguments=[
                    ('camera_name', 'camera'),
                    ('enable_rgbd', 'true'),
                    ('align_depth.enable', 'true'),
                    ('enable_sync', 'true'),
                    ('enable_color', 'true'),
                    ('enable_depth', 'true'),
                    ('pointcloud.enable', 'true'),
                ],
                condition=IfCondition(realsense_enable)
            )]
        ),

        # ── 7. LEAP MOTION HAND SENSOR (Delayed 16.0s) ─────────────────────────
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='leap_node',
                    executable='joint_publisher',
                    name='leapmotion_joint_publisher',
                    output='screen',
                    condition=IfCondition(leapmotion_enable),
                ),
                Node(
                    package='leap_node',
                    executable='image_publisher',
                    name='leapmotion_image_publisher',
                    output='screen',
                    condition=IfCondition(leapmotion_enable),
                )
            ]
        ),

        # ── 8. VICON BRIDGE SYSTEM (Delayed 19.0s) ─────────────────────────────
        TimerAction(
            period=19.0,
            actions=[Node(
                package='vicon_bridge',
                executable='vicon_bridge',
                name='vicon_bridge',
                parameters=[
                    {"host_name": vicon_host},
                    {"stream_mode": "ClientPull"},
                    {"world_frame_id": "map"},
                    {"tf_namespace": "vicon"}
                ],
                condition=IfCondition(vicon_enable)
            )]
        ),

        # ── 9. STANDARD USB WEB CAMERA (Delayed 22.0s) ─────────────────────────
        TimerAction(
            period=22.0,
            actions=[Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                output='screen',
                condition=IfCondition(camera_enable)
            )]
        ),

        # ── 10. TEMPLATE PLATFORM DESCRIPTION (Delayed 25.0s) ─────────────────
        TimerAction(
            period=25.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_platform_description),
                launch_arguments=[('urdf_file', 'default_template.urdf.xml')],
                condition=IfCondition(platform_description_enable)
            )]
        ),

        # ── 11. LASER SCAN CONVERTER FOR Lidar (Delayed 28.0s) ────────────────
        TimerAction(
            period=28.0,
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
                    ('cloud_in', 'livox/lidar'),  # maps Livox by default
                    ('scan', 'livox/scan')
                ],
            )]
        ),

        # ── 12. RViz2 GUI VISUALIZER (Delayed 31.0s) ──────────────────────────
        TimerAction(
            period=31.0,
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

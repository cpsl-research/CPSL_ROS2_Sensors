from ament_index_python.packages import get_package_share_directory
import os
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
pkg_platform_descriptions = get_package_share_directory('platform_descriptions')

ARGUMENTS = [
    DeclareLaunchArgument('urdf_file', default_value='create_3.urdf.xml',
                          description='the urdf file representing the robot description'),
    DeclareLaunchArgument('tf_prefix', default_value='',
                          description='tf_prefix to apply to frame id')
]


def launch_setup(context, *args, **kwargs):

    #load parameters
    urdf_file = LaunchConfiguration('urdf_file')
    tf_prefix = LaunchConfiguration('tf_prefix')

    #updating paths
    urdf_path = os.path.join(
        pkg_platform_descriptions,'urdf',urdf_file.perform(context)
    )

    #get the prefex string for the tf frames
    tf_prefex_str = tf_prefix.perform(context)
    if (tf_prefex_str):
        tf_prefex_str = "{}/".format(tf_prefex_str)
    else:
        tf_prefex_str = ""

    #read the urdf file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # rviz_config_file = PathJoinSubstitution([pkg_cpsl_navigation, 'rviz_cfgs', 'slam.rviz'])

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_desc,
                    'frame_prefix':tf_prefex_str
                }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        )
    ])

    return [bringup_group]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
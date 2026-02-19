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
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


pkg_dataset_generator = get_package_share_directory("dataset_generator")

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('param_file', default_value='ugv_dataset.yaml',
                          description='.yaml config file in the configs folder'),
    DeclareLaunchArgument('dataset_subpath', default_value='',
                          description='subpath to append to dataset_path')
]  

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    dataset_subpath = LaunchConfiguration('dataset_subpath')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str and not namespace_str.startswith('/')):
        namespace_str = '/' + namespace_str
    
    param_file_str = param_file.perform(context)
    dataset_subpath_str = dataset_subpath.perform(context)

    # Resolve the actual path to the parameter file
    actual_param_file_path = os.path.join(pkg_dataset_generator, 'configs', param_file_str)
    param_file_path = PathJoinSubstitution([pkg_dataset_generator, 'configs', param_file_str])

    param_substitutions = {}

    # If dataset_subpath is provided, update the dataset_path parameter
    if dataset_subpath_str:
        try:
            with open(actual_param_file_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Extract current dataset_path (assuming standard structure)
            # ROS2 params are usually under node_name: ros__parameters: 
            # In this case, the node name in the yaml is 'dataset_generator'
            current_path = config['dataset_generator']['ros__parameters']['dataset_path']
            new_path = os.path.join(current_path, dataset_subpath_str)
            param_substitutions['dataset_path'] = new_path
        except Exception as e:
            print(f"Error updating dataset_path with subpath: {e}")

    #update the parameter file with 
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        #launch the point cloud combiner node
        Node(
            package='dataset_generator',
            executable='dataset_generator',
            name='dataset_generator',
            output='screen',
            parameters=[configured_params],
        ),
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

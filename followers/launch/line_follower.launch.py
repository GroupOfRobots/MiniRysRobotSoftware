import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, actions, substitutions
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('followers'),
        'config',
        'params.yaml'
        )
    namespace_value = substitutions.LaunchConfiguration('minirys_namespace')
    arg_namespace = actions.DeclareLaunchArgument('minirys_namespace', default_value='minirys')
    node=Node(
        package = 'followers',
        name = 'line_follower',
        executable = 'linefollower.py',
        parameters = [config, {'minirys_namespace': namespace_value}]
    )
    ld = LaunchDescription([ arg_namespace, node])
    return ld

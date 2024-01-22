from launch import actions, substitutions, LaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    namespace_value = substitutions.LaunchConfiguration('namespace')

    arg_namespace = actions.DeclareLaunchArgument('namespace', default_value='minirys')

    node=Node(
    package='ros2_rpi_video_recorder',
    executable='ros2_rpi_video_recorder',
    name='video_recorder',
    namespace=namespace_value,
    )

    return LaunchDescription([arg_namespace,node])

import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription, substitutions
from launch_ros.actions import Node


def generate_launch_description():
    namespace_value = os.environ.get("NAMESPACE")
    config = os.path.join(
        get_package_share_directory("shuttlecock_detector_tree"),
        "config",
        "params.yaml",
    )
    servers = Node(
        package="shuttlecock_detector_tree",
        name="server_executor",
        executable="server_executor",
        parameters=[config],
    )
    clients = Node(
        package="shuttlecock_detector_tree",
        name="client_executor",
        executable="client_executor",
        parameters=[config],
    )

    ld = LaunchDescription([servers, clients])
    return ld

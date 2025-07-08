import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_value = os.environ.get('NAMESPACE')

    config_path = os.path.join(
        get_package_share_directory('minirys_camera'),
        'config',
        'simple_image_publisher_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'high_res_topic',
            default_value='internal/camera',
            description='High resolution image topic to publish on'
        ),
        DeclareLaunchArgument(
            'low_res_topic',
            default_value='internal/camera_low_res',
            description='Low resolution image topic to publish on'
        ),

        Node(
            package='minirys_camera',
            executable='simple_image_publisher',
            namespace=namespace_value,
            parameters=[config_path],
            remappings=[
                ('~/output/camera',         LaunchConfiguration('high_res_topic')),
                ('~/output/camera_low_res', LaunchConfiguration('low_res_topic')),
            ]
        )
    ])

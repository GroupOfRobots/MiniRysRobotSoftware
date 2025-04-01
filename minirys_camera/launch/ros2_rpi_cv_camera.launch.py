from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='minirys_camera',
            executable='ros2_rpi_cv_camera',
            name='cv_camera',
            parameters=[
                {'width': 640},
                {'height': 480},
                {'frame_interval': 0.5}
            ]
        ),
    ])

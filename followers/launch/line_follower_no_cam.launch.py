import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('followers'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'followers',
        name = 'line_follower_no_cam',
        executable = 'linefollower_no_cam.py',
        parameters = [config]
    )

    camera = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('ros2_rpi_cv_camera'),
				'ros2_rpi_cv_camera.launch.py')
		)
	)
    ld.add_action(node)
    ld.add_action(camera)
    return ld

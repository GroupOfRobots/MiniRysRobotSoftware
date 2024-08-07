import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription,  substitutions
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('shuttlecock_detect'),
        'config',
        'params.yaml'
        )
    node=Node(
        package = 'shuttlecock_detect',
        name = 'detector',
        executable = 'detector',
        parameters = [config]
    )

    camera = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_camera'),
				'ros2_rpi_cv_camera.launch.py')
		)
	)
    ld = LaunchDescription([ node, camera])
    return ld

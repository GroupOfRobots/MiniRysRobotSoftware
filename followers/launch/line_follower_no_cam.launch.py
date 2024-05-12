import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription,  substitutions
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('followers'),
        'config',
        'params.yaml'
        )
    namespace_value = substitutions.LaunchConfiguration('minirys_namespace')
    arg_namespace = actions.DeclareLaunchArgument('minirys_namespace', default_value='minirys')
    node=Node(
        package = 'followers',
        name = 'line_follower_no_cam',
        executable = 'linefollower_no_cam.py',
        parameters = [config, {'minirys_namespace': namespace_value}]
    )

    camera = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('ros2_rpi_cv_camera'),
				'ros2_rpi_cv_camera.launch.py')
		)
	)
    ld = LaunchDescription([arg_namespace, node, camera])
    return ld

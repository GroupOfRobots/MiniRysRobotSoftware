import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription,  substitutions
from launch_ros.actions import Node

def generate_launch_description():

    wall_f = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('followers'),
				'launch/wall_follower.launch.py')
		)
	)
	
    line_f = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('followers'),
				'launch/line_follower.launch.py')
		)
	)
        
    namespace_value = substitutions.LaunchConfiguration('minirys_namespace')
    arg_namespace = actions.DeclareLaunchArgument('minirys_namespace', default_value='minirys')
    node=Node(
        package = 'followers',
        name = 'combined_follower',
        executable = 'combinedFollower',
        parameters = [{'minirys_namespace': namespace_value}],
        
    )

    ld = LaunchDescription([arg_namespace, line_f, node, wall_f])
    return ld
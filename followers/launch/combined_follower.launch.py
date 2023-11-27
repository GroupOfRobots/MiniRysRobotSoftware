import os
from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
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
        
        
    node=Node(
        package = 'followers',
        name = 'combined_follower',
        executable = 'combinedFollower',
        
    )
    ld.add_action(line_f)
    ld.add_action(wall_f)
    ld.add_action(node)

    
    return ld

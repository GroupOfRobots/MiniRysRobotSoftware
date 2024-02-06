import os

from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription

def generate_launch_description():
	minirys_m = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_m.launch.py')
		)
	)

	minirys_cs = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_cs.launch.py')
			)
	)

	minirys_ve = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_ve.launch.py')
			)
	)

	minirys_vr = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_vr.launch.py')
		)
	)

	return LaunchDescription([
		minirys_m,
		minirys_cs,
		minirys_ve,
		minirys_vr,
	])

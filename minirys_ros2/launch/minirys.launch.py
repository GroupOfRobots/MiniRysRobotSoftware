import os

from ament_index_python.packages import get_package_share_directory
from launch import actions, launch_description_sources, LaunchDescription

def generate_launch_description():
	minirys_cs = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_cs_lidar.launch.py')
			)
	)

	minirys_ve = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_ve_lidar.launch.py')
			)
	)

	minirys_vr_lidar = actions.IncludeLaunchDescription(
		launch_description_sources.PythonLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('minirys_ros2'),
				'launch/minirys_vr_lidar.launch.py')
		)
	)

	return LaunchDescription([
		minirys_cs,
		minirys_ve,
		minirys_vr_lidar,
	])

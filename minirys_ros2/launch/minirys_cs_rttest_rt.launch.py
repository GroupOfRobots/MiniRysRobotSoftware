import os

from ament_index_python.packages import get_package_share_directory
from launch import actions, events, event_handlers, substitutions, LaunchDescription
import launch_ros

def get_shutdown_on_exit(action):
	return actions.RegisterEventHandler(event_handler=event_handlers.OnProcessExit(
		target_action=action,
		on_exit=[actions.EmitEvent(event=events.Shutdown())]
	))

def generate_launch_description():
	namespace_value = substitutions.LaunchConfiguration('namespace')

	params_path = os.path.join(
		get_package_share_directory('minirys_ros2'),
		'config',
		'params.yaml'
	)

	arg_namespace = actions.DeclareLaunchArgument('namespace', default_value='minirys')
	node_cs_rttest = launch_ros.actions.Node(
		package="minirys_ros2",
		executable="minirys_cs_rttest_rt",
		namespace=namespace_value,
		parameters=[params_path],
	)

	return LaunchDescription([
		arg_namespace,
		node_cs_rttest,
		get_shutdown_on_exit(node_cs_rttest),
	])

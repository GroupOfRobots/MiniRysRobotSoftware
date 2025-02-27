#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/GrabberControllerNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(2, 98, SCHED_RR);

	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto grabberControllerNode = GrabberControllerNode::make_shared(options);
	executor.add_node(grabberControllerNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}


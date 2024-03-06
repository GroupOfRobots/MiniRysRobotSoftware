#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/DistanceNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(2, 98, SCHED_RR);
	rclcpp::init(argc, argv);

	rclcpp::executors::MultiThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	// auto distanceNode = std::make_shared<DistanceNode>(options);
	// executor.add_node(distanceNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

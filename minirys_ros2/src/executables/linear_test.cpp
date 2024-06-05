#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/test/LinearTestNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(2, 98, SCHED_RR);

	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto linearTestNode = LinearTestNode::make_shared(options);
	executor.add_node(linearTestNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

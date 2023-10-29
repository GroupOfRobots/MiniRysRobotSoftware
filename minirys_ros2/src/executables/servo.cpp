#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/ServoNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(3, 98, SCHED_RR);
	
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto servoNode = ServoNode::make_shared(options);
	executor.add_node(servoNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

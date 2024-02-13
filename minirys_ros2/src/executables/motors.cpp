#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/MotorsNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(3, 98, SCHED_RR);
    // initiate communication interfaces
    // if (wiringPiSetup() == -1) {
    //     fprintf(stderr, "Not able to init the wiringPi library\n");
    //     return -1;
    // }
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto motorsNode = MotorsNode::make_shared(options);
	executor.add_node(motorsNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

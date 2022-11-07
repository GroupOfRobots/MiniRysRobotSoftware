#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/IMUNode.hpp"

#include <I2CBus.hpp>

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(2, 98, SCHED_RR);
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	auto i2c = I2CBus::makeShared("/dev/i2c-1");

	auto imuNode = std::make_shared<IMUNode>(i2c, options);
	executor.add_node(imuNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/CommunicationNode.hpp"
#include "minirys_ros2/nodes/FanRegulatorNode.hpp"
#include "minirys_ros2/nodes/ServoControllerNode.hpp"
#include "minirys_ros2/nodes/MotorsControllerNode.hpp"
#include "minirys_ros2/nodes/OdometryNode.hpp"

int main(int argc, char const* argv[]) {
	RTTExecutor::setupRT(1, 98, SCHED_RR);
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor executor;
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	//auto communicationNode = CommunicationNode::make_shared(options);
	auto fanRegulatorNode = FanRegulatorNode::make_shared(options);
	auto servoControllerNode = ServoControllerNode::make_shared(options);
	auto motorsControllerNode = MotorsControllerNode::make_shared(options);
	auto odometryNode = OdometryNode::make_shared(options);

	//executor.add_node(communicationNode);
	executor.add_node(fanRegulatorNode);
	executor.add_node(servoControllerNode);
	executor.add_node(motorsControllerNode);
	executor.add_node(odometryNode);

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

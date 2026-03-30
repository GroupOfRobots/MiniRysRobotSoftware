#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/CommunicationNode.hpp"
#include "minirys_ros2/nodes/FanRegulatorNode.hpp"
#include "minirys_ros2/nodes/MotorsControllerNode.hpp"
#include "minirys_ros2/nodes/OdometryNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rttest/rttest.h>

#include <chrono>

// NOLINTNEXTLINE(bugprone-exception-escape)
int main(int argc, char* argv[]) {
	RTTExecutor::setupRT(-1, 0, SCHED_OTHER);
	rclcpp::init(argc, argv);
	auto executor = RTTExecutor();

	rclcpp::NodeOptions nodeOptions;
	nodeOptions.use_intra_process_comms(true);
	auto communicationNode = CommunicationNode::make_shared(nodeOptions);
	auto fanRegulatorNode = FanRegulatorNode::make_shared(nodeOptions);
	auto motorsControllerNode = MotorsControllerNode::make_shared(nodeOptions);
	auto odometryNode = OdometryNode::make_shared(nodeOptions);

	executor.add_node(communicationNode);
	executor.add_node(fanRegulatorNode);
	executor.add_node(motorsControllerNode);
	executor.add_node(odometryNode);

	char filename[34] = "/tmp/minirys_cs_nonrt.results.txt";
	executor.init(120000, filename, false);
	rclcpp::install_signal_handlers();

	executor.spin();
	rclcpp::shutdown();
	return 0;
}

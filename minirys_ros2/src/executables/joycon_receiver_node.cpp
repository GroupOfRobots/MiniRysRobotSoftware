#include "rclcpp/rclcpp.hpp"
#include "minirys_ros2/nodes/JoyconReceiverNode.hpp"
#include "minirys_ros2/helpers/RTTExecutor.hpp"
//#include "rt.h"

int main(int argc, char const *argv[])
{
	// set real time priority for thread
	//setRTPriority("Joycon");

	// initiate ROS2
    RTTExecutor::setupRT(1, 98, SCHED_RR);
	rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto joycon = JoyconReceiverNode::make_shared(options);
    executor.add_node(joycon);

    executor.spin();
    rclcpp::shutdown();

	return 0;
}

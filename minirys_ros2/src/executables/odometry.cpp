#include "minirys_ros2/helpers/RTTExecutor.hpp"
#include "minirys_ros2/nodes/OdometryNode.hpp"

int main(int argc, char const* argv[]) {
    RTTExecutor::setupRT(3, 98, SCHED_RR);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);

    auto odometryNode = OdometryNode::make_shared(options);
    executor.add_node(odometryNode);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
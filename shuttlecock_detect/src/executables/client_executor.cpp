#include <string>
#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "shuttlecock_detect/actions/DeliverShuttlecockAction.hpp"
#include "shuttlecock_detect/actions/BackUpAction.hpp"
#include "shuttlecock_detect/actions/GetToShuttlecockAction.hpp"
#include "shuttlecock_detect/actions/PickShuttlecockAction.hpp"
#include "shuttlecock_detect/actions/RotateAction.hpp"
#include "shuttlecock_detect/actions/SearchingShuttlecockAction.hpp"
#include <chrono>
#include <thread>


using namespace std;
using namespace BT;


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    BehaviorTreeFactory factory;
    // BackUpAction
    auto bn = std::make_shared<rclcpp::Node>("back_up_client");
    RosNodeParams paramsBn;
    paramsBn.nh = bn;
    paramsBn.default_port_value = "back_up_service";
    factory.registerNodeType<BackUpAction>("BackUpAction", paramsBn);
    // DeliverShuttlecockAction
    auto dn = std::make_shared<rclcpp::Node>("deliver_shuttlecock_client");
    RosNodeParams paramsDn;
    paramsDn.nh = dn;
    paramsDn.default_port_value = "deliver_shuttlecock_service";
    factory.registerNodeType<DeliverShuttlecockAction>("DeliverShuttlecockAction", paramsDn);
    // GetToShuttlecockAction
    auto gn = std::make_shared<rclcpp::Node>("get_to_shuttlecock_client");
    RosNodeParams paramsGn;
    paramsGn.nh = gn;
    paramsGn.default_port_value = "get_to_shuttlecock_service";
    factory.registerNodeType<GetToShuttlecockAction>("GetToShuttlecockAction", paramsGn);
    // PickShuttlecockAction
    auto pn = std::make_shared<rclcpp::Node>("pick_shuttlecock_client");
    RosNodeParams paramsPn;
    paramsPn.nh = pn;
    paramsPn.default_port_value = "pick_shuttlecock_service";
    factory.registerNodeType<PickShuttlecockAction>("PickShuttlecockAction", paramsPn);
    // RotateAction
    auto rn = std::make_shared<rclcpp::Node>("rotate_client");
    RosNodeParams paramsRn;
    paramsRn.nh = rn;
    paramsRn.default_port_value = "rotate_service";
    factory.registerNodeType<RotateAction>("RotateAction", paramsRn);
    // SearchingShuttlecockAction
    auto sn = std::make_shared<rclcpp::Node>("searching_shuttlecock_client");
    RosNodeParams paramsSn;
    paramsSn.nh = sn;
    paramsSn.default_port_value = "searching_shuttlecock_service";
    factory.registerNodeType<SearchingShuttlecockAction>("SearchingShuttlecockAction", paramsSn);

    string package_share_directory = ament_index_cpp::get_package_share_directory("shuttlecock_detect");
        auto tree = factory.createTreeFromFile(package_share_directory+"/behavior_trees/detect_shuttlecock.xml");

    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
    }

    return 0;
}
#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/standard.hpp"

using namespace BT;

class BackUpAction : public RosActionNode<btcpp_ros2_interfaces::action::Standard>
{
public:
  BackUpAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::Standard>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
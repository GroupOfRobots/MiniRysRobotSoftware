#include "shuttlecock_detect/actions/DeliverShuttlecockAction.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool DeliverShuttlecockAction::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus DeliverShuttlecockAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus DeliverShuttlecockAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void DeliverShuttlecockAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(DeliverShuttlecockAction, "DeliverShuttlecockAction");

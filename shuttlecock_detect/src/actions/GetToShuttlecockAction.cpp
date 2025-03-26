#include "shuttlecock_detect/actions/GetToShuttlecockAction.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool GetToShuttlecockAction::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus GetToShuttlecockAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus GetToShuttlecockAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void GetToShuttlecockAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(GetToShuttlecockAction, "GetToShuttlecockAction");

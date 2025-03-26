#include "shuttlecock_detect/actions/BackUpAction.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool BackUpAction::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus BackUpAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus BackUpAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void BackUpAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(BackUpAction, "BackUpAction");
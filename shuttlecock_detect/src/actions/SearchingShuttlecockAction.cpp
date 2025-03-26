#include "shuttlecock_detect/actions/SearchingShuttlecockAction.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SearchingShuttlecockAction::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus SearchingShuttlecockAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->done ? "true" : "false");

  return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SearchingShuttlecockAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void SearchingShuttlecockAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

CreateRosNodePlugin(SearchingShuttlecockAction, "SearchingShuttlecockAction");


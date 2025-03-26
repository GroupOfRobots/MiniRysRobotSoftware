#pragma once

#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/standard.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

class BackUpServer : public rclcpp::Node
{
public:

  explicit BackUpServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Standard>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle);

  void execute(const std::shared_ptr<GoalHandleStandard> goal_handle);
  double timer_period_;
};
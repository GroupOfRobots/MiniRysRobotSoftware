#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/standard.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include <std_msgs/msg/bool.hpp>

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_cancel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_;
  double linear_speed_;
  double back_up_dist_;
  double rotate_speed_;
};
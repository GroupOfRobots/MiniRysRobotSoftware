#pragma once

#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/standard.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "btcpp_ros2_interfaces/msg/distances_and_transform.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_ros/transform_listener.h>

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

class GetToShuttlecockServer : public rclcpp::Node
{
public:

  explicit GetToShuttlecockServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Standard>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle);
  void execute(const std::shared_ptr<GoalHandleStandard> goal_handle);
  void distance_callback(const btcpp_ros2_interfaces::msg::DistancesAndTransform::SharedPtr msg);
  void send_goal();
  std::pair<float, float> calculate_dist();
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_;
  rclcpp::Subscription<btcpp_ros2_interfaces::msg::DistancesAndTransform>::SharedPtr subscription_dat_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_isCoverage_;
  float distance_;
  float deltX_;
  float stop_from_planner_;
  double timer_period_;
  geometry_msgs::msg::TransformStamped transform_;
};
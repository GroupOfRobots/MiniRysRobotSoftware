#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "btcpp_ros2_interfaces/action/standard.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

class DeliverShuttlecockServer : public rclcpp::Node
{
public:

  explicit DeliverShuttlecockServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Standard>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle);
  void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void execute(const std::shared_ptr<GoalHandleStandard> goal_handle);
  void deliver_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
  bool isRobotCloseToPose(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_delivery_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_;
  geometry_msgs::msg::PoseStamped msg_;
  bool is_goal_reached_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
  double timer_period_;
  double position_tolerance_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
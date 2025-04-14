#include "shuttlecock_detect/servers/DeliverShuttlecockServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

DeliverShuttlecockServer::DeliverShuttlecockServer(const rclcpp::NodeOptions& options)
  : Node("deliver_shuttlecock_action_server", options)
{
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Standard>(
      this, "deliver_shuttlecock_service", std::bind(&DeliverShuttlecockServer::handle_goal, this, _1, _2),
      std::bind(&DeliverShuttlecockServer::handle_cancel, this, _1),
      std::bind(&DeliverShuttlecockServer::handle_accepted, this, _1));

  this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
  this->declare_parameter("position_tolerance", rclcpp::ParameterValue(0.1));

  timer_period_ = this->get_parameter("timer_period").as_double();
  position_tolerance_ = this->get_parameter("timer_period").as_double();

  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Got param: position_tolerance " << position_tolerance_);
  
  subscription_delivery_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "delivery", 10, std::bind(&DeliverShuttlecockServer::deliver_callback, this, std::placeholders::_1));

  goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
    "navigate_to_pose/_action/status", 10,
     std::bind(&DeliverShuttlecockServer::status_callback, this, std::placeholders::_1));

  publisher_goal_= this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  std::string ns = this->get_namespace();
  ns.erase(0, 1);
  msg_ = geometry_msgs::msg::PoseStamped();
  msg_.pose.orientation.x = 0.0;
  msg_.pose.orientation.y = 0.0;
  msg_.pose.orientation.z = 1.0;
  msg_.pose.orientation.w = 0.0;
  msg_.header.frame_id = ns+"/map";
  is_goal_reached_ = false;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

rclcpp_action::GoalResponse DeliverShuttlecockServer::handle_goal(const rclcpp_action::GoalUUID&,
                                        std::shared_ptr<const Standard::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request ");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
DeliverShuttlecockServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DeliverShuttlecockServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&DeliverShuttlecockServer::execute, this, _1), goal_handle }.detach();
}

void DeliverShuttlecockServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  auto feedback = std::make_shared<Standard::Feedback>();
  auto result = std::make_shared<Standard::Result>();
  msg_.header.stamp = this->get_clock()->now();
  publisher_goal_->publish(msg_);
  is_goal_reached_ = false;
  while(!is_goal_reached_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(timer_period_*1000.0)));
    try
      {
          auto transform = tf_buffer_->lookupTransform("minirys2/map", "minirys2/base_link", tf2::TimePointZero);
          geometry_msgs::msg::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation = transform.transform.rotation;
          if(isRobotCloseToPose(pose, msg_)) break;

      }
    catch (...)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Problems with TF" );
    }

  }
  // Check if goal is done
  if(rclcpp::ok())
  {
    result->done = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void DeliverShuttlecockServer::deliver_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
  msg_ = *msg;
}

void DeliverShuttlecockServer::status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) 
{
  for (const auto & status : msg->status_list) {
    if (!(status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
      status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
      status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED))
    {
      is_goal_reached_ = false;
      return;
    }
  }
  is_goal_reached_ = true;
}

double DeliverShuttlecockServer::distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool DeliverShuttlecockServer::isRobotCloseToPose(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose)  
{
    double dist = distance(current_pose.position, target_pose.pose.position);
    return (dist <= position_tolerance_);
}
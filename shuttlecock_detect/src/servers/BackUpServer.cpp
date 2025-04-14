#include "shuttlecock_detect/servers/BackUpServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

  BackUpServer::BackUpServer(const rclcpp::NodeOptions& options)
    : Node("back_up_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Standard>(
        this, "back_up_service", std::bind(&BackUpServer::handle_goal, this, _1, _2),
        std::bind(&BackUpServer::handle_cancel, this, _1),
        std::bind(&BackUpServer::handle_accepted, this, _1));

    this->declare_parameter("linear_speed", rclcpp::ParameterValue(0.0));
    this->declare_parameter("rotate_speed", rclcpp::ParameterValue(0.5));
    this->declare_parameter("back_up_dist", rclcpp::ParameterValue(0.05));
 
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    rotate_speed_ = this->get_parameter("rotate_speed").as_double();
    back_up_dist_ = this->get_parameter("back_up_dist").as_double();
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << linear_speed_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: back_up_dist " << back_up_dist_);

    //publishers
    publisher_velocity_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }

  rclcpp_action::GoalResponse BackUpServer::handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  BackUpServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void BackUpServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{ std::bind(&BackUpServer::execute, this, _1), goal_handle }.detach();
  }

  void BackUpServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    action_client_->async_cancel_all_goals();
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();
    auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
    msg_twist->linear.x = -linear_speed_;
    publisher_velocity_->publish(*msg_twist);
    double time_end = back_up_dist_/linear_speed_;
    for(int i=0 ; i < (int)(time_end*2.0f); ++i )
    {
      publisher_velocity_->publish(*msg_twist);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    msg_twist->linear.x = 0.0;
    msg_twist->angular.z = rotate_speed_;
    double time_end_rotate = 3.14/rotate_speed_;
    publisher_velocity_->publish(*msg_twist);
    for(int i=0 ; i < (int)(time_end_rotate*2.0f); ++i )
    {
      publisher_velocity_->publish(*msg_twist);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if(rclcpp::ok())
    {
      msg_twist->angular.z = 0.0;
      publisher_velocity_->publish(*msg_twist);
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
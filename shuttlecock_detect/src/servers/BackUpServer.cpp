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
    rclcpp::Rate loop_rate(5);
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();
    // Check if goal is done
    if(rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
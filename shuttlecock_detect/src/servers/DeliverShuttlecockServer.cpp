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
    rclcpp::Rate loop_rate(5);
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();
    while(true)
    {
      
    }
    // Check if goal is done
    if(rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
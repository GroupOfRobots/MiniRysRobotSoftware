#include "shuttlecock_detect/servers/PickShuttlecockServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

  PickShuttlecockServer::PickShuttlecockServer(const rclcpp::NodeOptions& options)
    : Node("pick_shuttlecock_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Standard>(
        this, "pick_shuttlecock_service", std::bind(&PickShuttlecockServer::handle_goal, this, _1, _2),
        std::bind(&PickShuttlecockServer::handle_cancel, this, _1),
        std::bind(&PickShuttlecockServer::handle_accepted, this, _1));
    
    //load parameters
    this->declare_parameter("stop_docking", rclcpp::ParameterValue(0.0));
    this->declare_parameter("linear_speed", rclcpp::ParameterValue(0.0));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
 
    this->linear_speed_ = this->get_parameter("linear_speed").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    stop_docking_ = (float) this->get_parameter("stop_docking").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << linear_speed_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: stop_docking " << stop_docking_);

    //publishers
    publisher_velocity_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //subscribers
    subscription_dat_ = this->create_subscription<btcpp_ros2_interfaces::msg::DistancesAndTransform>(
        "distances", 10, std::bind(&PickShuttlecockServer::distance_callback, this, std::placeholders::_1));

    distance_ = -1.0;
  }

  rclcpp_action::GoalResponse PickShuttlecockServer::handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  PickShuttlecockServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void PickShuttlecockServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&PickShuttlecockServer::execute, this, _1), goal_handle }.detach();
  }

  void PickShuttlecockServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();

    while(rclcpp::ok())
    {
      if(distance_ != -1.0)
      {
          if(distance_ > stop_docking_)
          {
              auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
              msg_twist->linear.x = linear_speed_;
              publisher_velocity_->publish(*msg_twist);
          }
          else
          {
              auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
              publisher_velocity_->publish(*msg_twist);
              result->done = true;
              goal_handle->succeed(result);
              RCLCPP_INFO(this->get_logger(), "Goal succeeded");
              break;
          }
      }
      else
      {
          auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
          publisher_velocity_->publish(*msg_twist);
          result->done = false;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal fail");
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(timer_period_*1000.0f)));
    }
  }

void PickShuttlecockServer::distance_callback(const btcpp_ros2_interfaces::msg::DistancesAndTransform::SharedPtr msg) 
{
    distance_ = msg->distance;
    deltX_ = msg->delt_x;
    transform_=msg->transform;
}

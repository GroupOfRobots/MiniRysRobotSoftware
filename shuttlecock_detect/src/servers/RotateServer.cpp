#include "shuttlecock_detect/servers/RotateServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

  RotateServer::RotateServer(const rclcpp::NodeOptions& options )
    : Node("rotate_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Standard>(
        this, "rotate_service", std::bind(&RotateServer::handle_goal, this, _1, _2),
        std::bind(&RotateServer::handle_cancel, this, _1),
        std::bind(&RotateServer::handle_accepted, this, _1));

        //declare parameters
    this->declare_parameter("stop_rotate", rclcpp::ParameterValue(0.0));
    this->declare_parameter("K", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Ti", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Td", rclcpp::ParameterValue(0.0));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    
    //load parameters
    double K = this->get_parameter("K").as_double();
    double Ti = this->get_parameter("Ti").as_double();
    double Td = this->get_parameter("Td").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    stop_rotate_ = (float) this->get_parameter("stop_rotate").as_double();
    this->pid_ = std::unique_ptr<PIDRegulator>(new PIDRegulator(timer_period_,
        (float) K,(float) Ti,(float) Td));

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Ti " << Ti);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Td " << Td);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: K " << K);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: stop_rotate " << stop_rotate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period_);

    //publishers
    publisher_isCoverage_ =  this->create_publisher<std_msgs::msg::Bool>("coverage", 10);
    publisher_vocity_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    //subscribers
    subscription_dat_ = this->create_subscription<btcpp_ros2_interfaces::msg::DistancesAndTransform>(
        "distances", 10, std::bind(&RotateServer::distance_callback, this, std::placeholders::_1));

    //actions
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    distance_ = -1.0;
  }

  rclcpp_action::GoalResponse RotateServer::handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  RotateServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void RotateServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&RotateServer::execute, this, _1), goal_handle }.detach();
  }

  void RotateServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();
    int rotateNoShCounter = 0;

    while(rclcpp::ok())
    {
      if(distance_ != -1.0)
      {
          if(std::abs(deltX_) > stop_rotate_)
          {
              auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
              msg_twist->angular.z = pid_->pid(deltX_,0.0f);
              publisher_vocity_->publish(*msg_twist);
          }
          else
          {
              result->done = true;
              goal_handle->succeed(result);
              RCLCPP_INFO(this->get_logger(), "Goal succeeded");
              auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
              publisher_vocity_->publish(*msg_twist);
              break;
          }
      }
      else
      {
          ++rotateNoShCounter;
      }
      if(rotateNoShCounter == 4)
      {
          auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
          publisher_vocity_->publish(*msg_twist);
          result->done = false;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal fail");
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

void RotateServer::distance_callback(const btcpp_ros2_interfaces::msg::DistancesAndTransform::SharedPtr msg) 
{
    distance_ = msg->distance;
    deltX_ = msg->delt_x;
    transform_=msg->transform;
}


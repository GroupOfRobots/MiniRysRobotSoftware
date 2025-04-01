#include "shuttlecock_detect/servers/SearchingShuttlecockServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

  SearchingShuttlecockServer::SearchingShuttlecockServer(const rclcpp::NodeOptions& options )
    : Node("searching_shuttlecock_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Standard>(
        this, "searching_shuttlecock_service", std::bind(&SearchingShuttlecockServer::handle_goal, this, _1, _2),
        std::bind(&SearchingShuttlecockServer::handle_cancel, this, _1),
        std::bind(&SearchingShuttlecockServer::handle_accepted, this, _1));

    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    timer_period_ = this->get_parameter("timer_period").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period_);

    //publishers
    publisher_goal_= this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    publisher_isCoverage_ =  this->create_publisher<std_msgs::msg::Bool>("coverage", 10);

    //subscribers
    subscription_dat_ = this->create_subscription<btcpp_ros2_interfaces::msg::DistancesAndTransform>(
        "distances", 10, std::bind(&SearchingShuttlecockServer::distance_callback, this, std::placeholders::_1));

    distance_ = -1.0;
  }

  rclcpp_action::GoalResponse SearchingShuttlecockServer::handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request searching shuttlecocks");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  SearchingShuttlecockServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void SearchingShuttlecockServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    using namespace std::placeholders;

    std::thread{ std::bind(&SearchingShuttlecockServer::execute, this, _1), goal_handle }.detach();
  }

  void SearchingShuttlecockServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    auto msg_bool = std_msgs::msg::Bool();
    msg_bool.data = true;
    publisher_isCoverage_->publish(msg_bool);
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();

    while(rclcpp::ok())
    {
      if(distance_ != -1.0)
      {
          RCLCPP_INFO(this->get_logger(), "Lotka");
          this->send_goal();
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    if(rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

void SearchingShuttlecockServer::distance_callback(const btcpp_ros2_interfaces::msg::DistancesAndTransform::SharedPtr msg) 
{
    distance_ = msg->distance;
    deltX_ = msg->delt_x;
    transform_=msg->transform;
}

void SearchingShuttlecockServer::send_goal()
{
    tf2::Quaternion q(
                    transform_.transform.rotation.x,
                    transform_.transform.rotation.y,
                    transform_.transform.rotation.z,
                    transform_.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double x_offset = distance_ * std::cos(yaw) + deltX_ * std::sin(yaw);
    double y_offset = distance_ * std::sin(yaw) - deltX_ * std::cos(yaw);
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "minirys2/map";
    msg.header.stamp = this->get_clock()->now();
    msg.pose.position.x = transform_.transform.translation.x + x_offset;
    msg.pose.position.y = transform_.transform.translation.y + y_offset;
    msg.pose.position.z = transform_.transform.translation.z;
    msg.pose.orientation = transform_.transform.rotation;
    publisher_goal_->publish(msg);
}
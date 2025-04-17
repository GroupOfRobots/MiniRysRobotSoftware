#include "shuttlecock_detect/servers/GetToShuttlecockServer.hpp"

using Standard = btcpp_ros2_interfaces::action::Standard;
using GoalHandleStandard = rclcpp_action::ServerGoalHandle<Standard>;

  GetToShuttlecockServer::GetToShuttlecockServer(const rclcpp::NodeOptions& options)
    : Node("get_to_shuttlecock_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Standard>(
        this, "get_to_shuttlecock_service", std::bind(&GetToShuttlecockServer::handle_goal, this, _1, _2),
        std::bind(&GetToShuttlecockServer::handle_cancel, this, _1),
        std::bind(&GetToShuttlecockServer::handle_accepted, this, _1));


    this->declare_parameter("stop_from_planner", rclcpp::ParameterValue(0.0));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    stop_from_planner_ = (float) this->get_parameter("stop_from_planner").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: stop_from_planner " << stop_from_planner_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period_);

    //publishers
    publisher_goal_= this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    publisher_isCoverage_ =  this->create_publisher<std_msgs::msg::Bool>("coverage", 10);

    //subscribers
    subscription_dat_ = this->create_subscription<btcpp_ros2_interfaces::msg::DistancesAndTransform>(
        "distances", 10, std::bind(&GetToShuttlecockServer::distance_callback, this, std::placeholders::_1));

    publisher_cancel_ = this->create_publisher<std_msgs::msg::Bool>(
    "stop_navigate", 
    10);

    distance_ = -1.0;
  }

  rclcpp_action::GoalResponse GetToShuttlecockServer::handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Standard::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sleep time ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  GetToShuttlecockServer::handle_cancel(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void GetToShuttlecockServer::handle_accepted(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{ std::bind(&GetToShuttlecockServer::execute, this, _1), goal_handle }.detach();
  }

  void GetToShuttlecockServer::execute(const std::shared_ptr<GoalHandleStandard> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<Standard::Feedback>();
    auto result = std::make_shared<Standard::Result>();
    int counter = 0, closer_counter = 0;
    float ori_dist = std::numeric_limits<float>::max();

    while(rclcpp::ok())
    {
      if(distance_ != -1.0)
      {
          this->send_goal();
          if(ori_dist >= distance_)
          {
            closer_counter = closer_counter +1;
            ori_dist = distance_;
          }
          if(counter > 4 && distance_ < stop_from_planner_)
          {
            result->done = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            std_msgs::msg::Bool cancel_msg;
            publisher_cancel_->publish(cancel_msg);
            break;
          }
          
      }
      else if (counter > 4)
      {
        result->done = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        std_msgs::msg::Bool cancel_msg;
        publisher_cancel_->publish(cancel_msg);
        break;
      }
      
      if(counter == 4)
      {
          if(closer_counter <= 1)
          {
            std_msgs::msg::Bool cancel_msg;
            publisher_cancel_->publish(cancel_msg);
            result->done = false;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal failed");
            break;
          }
          else{
            auto msg_bool = std_msgs::msg::Bool();
            msg_bool.data = false;
            publisher_isCoverage_->publish(msg_bool);
          }
      }
      ++counter;
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(timer_period_*1000.0)));
    }
  }

void GetToShuttlecockServer::distance_callback(const btcpp_ros2_interfaces::msg::DistancesAndTransform::SharedPtr msg) 
{
    distance_ = msg->distance;
    deltX_ = msg->delt_x;
    transform_=msg->transform;
}

void GetToShuttlecockServer::send_goal()
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
    std::string ns = this->get_namespace();
    ns.erase(0, 1);
    msg.header.frame_id = ns + "/map";
    msg.header.stamp = this->get_clock()->now();
    msg.pose.position.x = transform_.transform.translation.x + x_offset;
    msg.pose.position.y = transform_.transform.translation.y + y_offset;
    msg.pose.position.z = transform_.transform.translation.z;
    msg.pose.orientation = transform_.transform.rotation;
    publisher_goal_->publish(msg);
}
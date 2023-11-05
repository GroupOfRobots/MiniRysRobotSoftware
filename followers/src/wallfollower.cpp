#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "pid.cpp"
#include <memory>



using namespace std::chrono_literals;

class WallFollower: public rclcpp::Node{
    public:
    WallFollower() : Node("wall_follower") {

        this->declare_parameter("timer_period", rclcpp::ParameterValue(0.0));
        this->declare_parameter("K", rclcpp::ParameterValue(0.0));
        this->declare_parameter("Ti", rclcpp::ParameterValue(0.0));
        this->declare_parameter("Td", rclcpp::ParameterValue(0.0));
        this->declare_parameter("linearSpeed", rclcpp::ParameterValue(0.0));

        // Get and save/use the parameters
        std::this_thread::sleep_for(100ms);
        

        double timer_period = this->get_parameter("timer_period").as_double();
        double K = this->get_parameter("K").as_double();
        double Ti = this->get_parameter("Ti").as_double();
        double Td = this->get_parameter("Td").as_double();
        this->linearSpeed = this->get_parameter("linearSpeed").as_double();
        this->pid = std::unique_ptr<PID>(new PID((float) timer_period,(float) K,(float) Ti,(float) Td));

        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Ti " << Ti);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Td " << Td);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: K " << K);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << linearSpeed);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);

        publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/minirys/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&WallFollower::timer_callback, this));

        subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_5", 10, std::bind(&WallFollower::left_sensor_callback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_2", 10, std::bind(&WallFollower::right_sensor_callback, this, std::placeholders::_1));

    }

    private:

    void timer_callback() {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        float u = this->pid->pid(this->right_sensor-this->left_sensor,0);
        msg->linear.y = this->linearSpeed;
        msg->angular.z = u;
        publisher_->publish(*msg);
        
    }

    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->left_sensor = (float) msg->range;

    }

    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->right_sensor = (float) msg->range;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription2_;

    float left_sensor = 0.0f;
    float right_sensor = 0.0f;

    std::unique_ptr<PID> pid;
    double linearSpeed;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}

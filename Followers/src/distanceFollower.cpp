#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "pid.cpp"



using namespace std::chrono_literals;

class DistFollower: public rclcpp::Node{
    public:
    DistFollower() : Node("distance_follower") {

        publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/minirys/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&DistFollower::timer_callback, this));

        subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_5", 10, std::bind(&DistFollower::left_sensor_callback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_2", 10, std::bind(&DistFollower::right_sensor_callback, this, std::placeholders::_1));
    }

    private:

    void timer_callback() {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        float u = this->pid.pid(this->right_sensor-this->left_sensor,0);
	std::cout<<this->left_sensor-this->right_sensor<<std::endl;
        msg->linear.y = -1.0;
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
    PID pid = PID(0.05f,5.5f,100.0f,0.015f);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistFollower>());
  rclcpp::shutdown();
  return 0;
}

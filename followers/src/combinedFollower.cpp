#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>


using namespace std::chrono_literals;

class CombinedFollower: public rclcpp::Node{
public:
    CombinedFollower() : Node("combined_follower") {

        subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_5", 10, std::bind(&CombinedFollower::left_sensor_callback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_2", 10, std::bind(&CombinedFollower::right_sensor_callback, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_3", 10, std::bind(&CombinedFollower::front_sensor_callback, this, std::placeholders::_1));
        
        publisher1_ = this->create_publisher<std_msgs::msg::Bool>("/is_wall_follower", 10);
        publisher2_ = this->create_publisher<std_msgs::msg::Bool>("/is_line_follower", 10);
        publisher3_ = this->create_publisher<std_msgs::msg::Bool>("/minirys/balance_mode", 10);

        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&CombinedFollower::timer_callback, this));
        
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher1_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher3_;
    int flag_ = 0;
    float left_sensor = 400.0f;
    float right_sensor = 400.0f;
    float front_sensor = 400.0f;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_balancing;


    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->left_sensor = (float) msg->range;

    }

    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->right_sensor = (float) msg->range;
    }

    void front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->front_sensor = (float) msg->range;
    }

    void timer_callback() {
        auto msg = std::make_shared<std_msgs::msg::Bool>();
        if(flag_ == 0){
            msg->data = true;
            publisher2_->publish(*msg);
            msg->data = false;
            publisher1_->publish(*msg);
            flag_ = 1;
        }
        if(flag_ == 1){
            if(right_sensor < 0.200 && left_sensor < 0.2){
                flag_ = 2;
                msg->data = false;
                publisher2_->publish(*msg);
                msg->data = true;
                publisher3_->publish(*msg);
                start_balancing = std::chrono::high_resolution_clock::now();
            }
            if(std::chrono::duration_cast<std::chrono::milliseconds>(start_balancing - std::chrono::high_resolution_clock::now()).count() > 2000 && flag_==2){
                msg->data = true;
                publisher1_->publish(*msg);
                flag_ = 3;
            }

            if(right_sensor < 0.200 && left_sensor < 0.2 && front_sensor < 0.2 && flag_==3){
                msg->data = false;
                publisher1_->publish(*msg);
            }
        }

    };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombinedFollower>());
  rclcpp::shutdown();
  return 0;
}

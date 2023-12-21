#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>


using namespace std::chrono_literals;

class CombinedFollower: public rclcpp::Node{
public:
    CombinedFollower();
    
    ~CombinedFollower();
private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher1_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher3_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher4_;
    int flag_ = -1;
    float left_sensor = 400.0f;
    float right_sensor = 400.0f;
    float front_sensor = 400.0f;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_;

    int getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time);

    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    
    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    

    void front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    

    void timer_callback();
};



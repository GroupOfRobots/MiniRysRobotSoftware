#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>


using namespace std::chrono_literals;

enum CombinedFollowerStates{
    INITIAL_STATE,
    START_FOLLOWING_LINE,
    CORRIDOR_BEGIN,
    GO_STRAIGHT,
    STANDING_UP,
    START_FOLLOWING_WALL,
    STOP,
    TURN_BACK,
    STABILISE,
    CONTINUE_FOLLOWING_WALL,
    END_OF_CORRIDOR

};

class CombinedFollower: public rclcpp::Node{
public:
    CombinedFollower();
    
    ~CombinedFollower();
private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist5_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist3_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription_dist2_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool_wallf_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool_linef_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_bool_balance_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity_;
    CombinedFollowerStates flag_ = INITIAL_STATE;
    float left_sensor = 400.0f;
    float right_sensor = 400.0f;
    float front_sensor = 400.0f;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_;

    int getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time);
    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void timer_callback();
    float corridor_recogniton_dist = 0.0f;
    float end_side_dist = 0.0f;
    float end_front_dist = 0.0f;
    float corridor_out_front_dist = 0.0f;
    float corridor_out_side_dist = 0.0f;
    int stabilization_time = 0;
    int standing_up_time = 0;
    int turning_time = 0;
    int positioning_time = 0;
    const int INIT_TIME = 500;
    const float TURN_VAL = 1.57;
};
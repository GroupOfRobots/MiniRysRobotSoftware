#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include "shuttlecock_detect/helpers/Yolov7.hpp"
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <utility>
#include <iostream>
#include <math.h>

using namespace std::chrono_literals;

enum DetectorStates{
	DETECTING,
    GETTING_CLOSER,
    WAITING_FOR_ARRIVAL,
    DOCKING,
    GETTING_BACK
};

class Detector: public rclcpp::Node{
    public:

    Detector();

    private:

    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    std::pair<float, float> calculate_dist();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_detected_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;

    cv::Mat ori_img_;
    cv::Mat detected_img_;
    std::unique_ptr<YoloV7> yolov7_;
    float prob_threshold_;
    float nms_threshold_;
    float focal_length_;
    float width_side_;
    float width_front_;
    nav_msgs::msg::Odometry current_odom_;
    DetectorStates state_;
    int counter_;
    float ori_dist_;
    bool is_closer_;
};

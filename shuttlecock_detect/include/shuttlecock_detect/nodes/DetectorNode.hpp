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
#include <iostream>

using namespace std::chrono_literals;

enum DetectorStates{
	DETECTING,
    GETTING_CLOSER,
    DOCKING,
    GETTING_BACK
};

class Detector: public rclcpp::Node{
    public:

    Detector();

    private:

    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg); 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_detected_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_goal_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;

    cv::Mat ori_img_;
    cv::Mat detected_img_;
    std::unique_ptr<YoloV7> yolov7_;
    float prob_threshold_;
    float nms_threshold_;
    float focal_length_;
    float width_side_;
    float width_front_;
};

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

using namespace std::chrono_literals;

class Detector: public rclcpp::Node{
    public:

    Detector();
    ~WallFollower();

    private:

    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg); 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_detected_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    cv::Mat ori_img_;
    cv::Mat detected_img_;
    std::unique_ptr<YoloV7> yolov7_;
    float prob_threshold_;
    float nms_threshold_;
};

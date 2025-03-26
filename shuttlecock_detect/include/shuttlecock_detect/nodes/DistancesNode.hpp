#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "shuttlecock_detect/helpers/Yolov7.hpp"
#include <memory>
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <utility>
#include <iostream>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "btcpp_ros2_interfaces/msg/distances_and_transform.hpp"

using namespace std::chrono_literals;


class Distances: public rclcpp::Node{
    public:

    Distances();

    private:

    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    std::pair<float, float> calculate_dist();
    rclcpp::Publisher<btcpp_ros2_interfaces::msg::DistancesAndTransform>::SharedPtr publisher_dat_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    cv::Mat ori_img_;
    cv::Mat detected_img_;
    std::unique_ptr<YoloV7> yolov7_;
    float prob_threshold_;
    float nms_threshold_;
    float focal_length_;
    float width_side_;
    float width_front_;
    DetectorStates state_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
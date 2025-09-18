#pragma once

#include "btcpp_ros2_interfaces/msg/distances_and_transform.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shuttlecock_detect/helpers/Yolov7.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <math.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <utility>

using namespace std::chrono_literals;

class Distances : public rclcpp::Node {
public:
  Distances(rclcpp::NodeOptions options);

private:
  void timer_callback();
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::pair<float, float> calculate_dist();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_detected_;
  rclcpp::Publisher<btcpp_ros2_interfaces::msg::DistancesAndTransform>::
      SharedPtr publisher_dat_;
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
  float ccd_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
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
#include "minirys_ros2/helpers/PIDRegulator.hpp"

using namespace std::chrono_literals;

enum DetectorStates{
	DETECTING,
    GETTING_CLOSER,
    WAITING_FOR_ARRIVAL,
    DOCKING,
    ROTATING,
    GETTING_BACK,
    RETURN,
};

class Detector: public rclcpp::Node{
    public:

    Detector();

    private:

    void timer_callback();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void send_goal(const std::pair<float, float>& distances, const geometry_msgs::msg::TransformStamped& transform_stamped);
    std::pair<float, float> calculate_dist();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_detected_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_isCoverage_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vocity_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    cv::Mat ori_img_;
    cv::Mat detected_img_;
    std::unique_ptr<YoloV7> yolov7_;
    float prob_threshold_;
    float nms_threshold_;
    float focal_length_;
    float width_side_;
    float width_front_;
    float stop_from_planner_;
    float stop_rotate_;
    float stop_docking_;
    DetectorStates state_;
    int counter_;
    int closer_counter_;
    float ori_dist_;
    double linear_speed_;
    bool is_goal_reached_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<PIDRegulator> pid_;
    int rotateNoShCounter_;
};

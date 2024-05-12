#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <minirys_msgs/srv/set_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <cstdlib>

class OdometryNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(OdometryNode);

	explicit OdometryNode(rclcpp::NodeOptions options);

	~OdometryNode() override = default;

private:
	std::string envNamespace = std::getenv("NAMESPACE");
	double poseX;
	double poseY;
	double poseTheta;
	bool poseValid;

    bool invertLeftMotor;
    bool invertRightMotor;
	double wheelRadius;
	double wheelSeparation;
	double wheelRadiusCorrection;
	double wheelSeparationCorrection;

	double motorPositionL;
	double motorPositionR;
	double motorPositionLPrev;
	double motorPositionRPrev;
	double motorSpeedL;
	double motorSpeedR;

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr odometryValidPublisher;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorPositionLSubscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorPositionRSubscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorSpeedLSubscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorSpeedRSubscription;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusLSubscription;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusRSubscription;

	rclcpp::Service<minirys_msgs::srv::SetPose>::SharedPtr setPoseService;

	void update();

	void receiveMotorPositionL(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorPositionR(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorSpeedL(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorSpeedR(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorStatusL(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);

	void receiveMotorStatusR(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);

	void setPose(
		const minirys_msgs::srv::SetPose::Request::SharedPtr request,
		minirys_msgs::srv::SetPose::Response::SharedPtr response
	);
};

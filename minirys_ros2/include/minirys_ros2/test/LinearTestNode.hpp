#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class LinearTestNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(LinearTestNode);

	explicit LinearTestNode(rclcpp::NodeOptions options);

	~LinearTestNode() override = default;

private:
	double linearVelocity;
	double linearDistance;
    double updateFrequency;

    geometry_msgs::msg::Twist msg;

    nav_msgs::msg::Odometry::SharedPtr odometry;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityCommandPublisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber;

	void move_linear();

	void receiveOdometry(const nav_msgs::msg::Odometry::SharedPtr message);

};
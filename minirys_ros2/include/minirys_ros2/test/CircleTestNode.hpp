#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class CircleTestNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(CircleTestNode);

	explicit CircleTestNode(rclcpp::NodeOptions options);

	~CircleTestNode() override = default;

private:
	double angularVelocity;
	double numberOfCircles;
    double updateFrequency;
	bool turnLeft;

    geometry_msgs::msg::Twist msg;

    nav_msgs::msg::Odometry::SharedPtr odometry;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityCommandPublisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber;

	void move_circle();

	void receiveOdometry(const nav_msgs::msg::Odometry::SharedPtr message);

};
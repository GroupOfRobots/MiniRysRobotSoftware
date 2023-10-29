#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/angular_pose.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

class ServoControllerNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(ServoControllerNode);

	explicit ServoControllerNode(rclcpp::NodeOptions options);

	~ServoControllerNode() override = default;

private:
	double servoDutyUp;
	double servoDutyDown;
    double robotAngularPosition;

	bool servoStatus;
	bool balanceMode;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servoOutputPublisher;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr balanceModeSubscriber;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servoStatusSubscriber;

    rclcpp::Subscription<minirys_msgs::msg::AngularPose>::SharedPtr angularPoseSubscriber;



	void update();

	void receiveBalanceMode(const std_msgs::msg::Bool::SharedPtr message);

	void receiveAngularPose(const minirys_msgs::msg::AngularPose::SharedPtr message);

	void receiveServoStatus(const std_msgs::msg::Bool::SharedPtr message);

};

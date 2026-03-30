#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <PWMPin.hpp>

class GrabberNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(GrabberNode);

	explicit GrabberNode(rclcpp::NodeOptions options);

	~GrabberNode() override;

private:
	PWMPin::SharedPtr pwm;

	int pin;

	float output;

	float openDuty;

	float closeDuty;

    bool is_closed;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grabberStateSubscriber;

	rclcpp::TimerBase::SharedPtr updateTimer;

	void receiveGrabberState(const std_msgs::msg::Bool::SharedPtr message);
	void update();

};


#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <PWMPin.hpp>

class FanNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(FanNode);

	explicit FanNode(rclcpp::NodeOptions options);

	~FanNode() override;

private:
	PWMPin::SharedPtr pwm;

	float output;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fanOutputSubscriber;

	void update();

	void receiveFanOutput(const std_msgs::msg::Float32::SharedPtr message);
};

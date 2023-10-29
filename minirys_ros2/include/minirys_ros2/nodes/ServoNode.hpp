#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <PWMPin.hpp>

#include <bcm2835.hpp>

class ServoNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(ServoNode);

	explicit ServoNode(rclcpp::NodeOptions options);

	~ServoNode() override;

private:
	PWMPin::SharedPtr pwm;

	int pin;

	float output;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr servoOutputSubscriber;

	void update();

	void receiveServoOutput(const std_msgs::msg::Float32::SharedPtr message);

	void setDownPosition();

    void setUpPosition();
};

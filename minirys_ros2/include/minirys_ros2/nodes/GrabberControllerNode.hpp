#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <PWMPin.hpp>

class GrabberControllerNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(GrabberControllerNode);

	explicit GrabberControllerNode(rclcpp::NodeOptions options);

	~GrabberControllerNode() override;

private:
	PWMPin::SharedPtr pwm;

	int pin;

	float output;

	float openDuty;

	float closeDuty;

    bool is_open;

	rclcpp::TimerBase::SharedPtr updateTimer;

	void update();

};


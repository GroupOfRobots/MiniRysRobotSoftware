#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/motor_command.hpp>
#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <std_msgs/msg/float64.hpp>

#include <L6470.hpp>
#include <SPIBus.hpp>

class MotorsNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(MotorsNode);

	explicit MotorsNode(rclcpp::NodeOptions options);

	~MotorsNode() override;

private:
	double stepsPerRevolution;
	double maxSpeed;

	SPIBus::SharedPtr spi;

	GPIOPin::SharedPtr resetPin;

	GPIOPin::SharedPtr busyPin;

	L6470::SharedPtr motors;

	double speedL;

	double speedR;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorPositionLPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorPositionRPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorSpeedLPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorSpeedRPublisher;

	rclcpp::Publisher<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusLPublisher;

	rclcpp::Publisher<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusRPublisher;

	rclcpp::Subscription<minirys_msgs::msg::MotorCommand>::SharedPtr motorCommandSubscription;

	void update();

	void receiveMotorCommand(const minirys_msgs::msg::MotorCommand::SharedPtr message);
};

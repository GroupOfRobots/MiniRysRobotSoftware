#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/motor_command.hpp>
#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <motorsclass.h>


class MotorsNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(MotorsNode);

	explicit MotorsNode(rclcpp::NodeOptions options);

	~MotorsNode() override;

private:
    double speedL;

    double speedR;

	double stepsPerRevolution;

	double maxSpeed;

    double wheelRadius;

    std::shared_ptr<Motors> motors;

	rclcpp::TimerBase::SharedPtr updateTimer;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  jointPublisher;

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

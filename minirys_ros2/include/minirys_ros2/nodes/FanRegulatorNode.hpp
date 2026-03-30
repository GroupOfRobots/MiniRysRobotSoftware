#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <std_msgs/msg/float32.hpp>

class FanRegulatorNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(FanRegulatorNode);

	explicit FanRegulatorNode(rclcpp::NodeOptions options);

	~FanRegulatorNode() override = default;

private:
	double fanLevelLow;
	double fanLevelMedium;
	double fanLevelHigh;
	double thresholdCPUWarning;
	double thresholdCPUCritical;
	double thresholdMainWarning;
	double thresholdMainCritical;

	float tempCPU;

	float tempMain;

	bool motorLTempWarning;

	bool motorRTempWarning;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fanOutputPublisher;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tempMainSubscription;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr tempCPUSubscription;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusLSubscription;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusRSubscription;

	void update();

	void receiveTempMain(const std_msgs::msg::Float32::SharedPtr message);

	void receiveTempCPU(const std_msgs::msg::Float32::SharedPtr message);

	void receiveMotorLStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);

	void receiveMotorRStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);
};

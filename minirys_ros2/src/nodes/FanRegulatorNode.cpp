#include "minirys_ros2/nodes/FanRegulatorNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

FanRegulatorNode::FanRegulatorNode(rclcpp::NodeOptions options):
	Node("fan_regulator_cs", options),
	tempCPU(0),
	tempMain(0),
	motorLTempWarning(false),
	motorRTempWarning(false) {
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(2.0));
	this->declare_parameter("fanLevelLow", rclcpp::ParameterValue(30.0));
	this->declare_parameter("fanLevelMedium", rclcpp::ParameterValue(60.0));
	this->declare_parameter("fanLevelHigh", rclcpp::ParameterValue(100.0));
	this->declare_parameter("thresholdCPUWarning", rclcpp::ParameterValue(50.0));
	this->declare_parameter("thresholdCPUCritical", rclcpp::ParameterValue(65.0));
	this->declare_parameter("thresholdMainWarning", rclcpp::ParameterValue(45.0));
	this->declare_parameter("thresholdMainCritical", rclcpp::ParameterValue(60.0));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	this->fanLevelLow = this->get_parameter("fanLevelLow").as_double();
	this->fanLevelMedium = this->get_parameter("fanLevelMedium").as_double();
	this->fanLevelHigh = this->get_parameter("fanLevelHigh").as_double();
	this->thresholdCPUWarning = this->get_parameter("thresholdCPUWarning").as_double();
	this->thresholdCPUCritical = this->get_parameter("thresholdCPUCritical").as_double();
	this->thresholdMainWarning = this->get_parameter("thresholdMainWarning").as_double();
	this->thresholdMainCritical = this->get_parameter("thresholdMainCritical").as_double();

	this->tempMainSubscription = this->create_subscription<std_msgs::msg::Float32>(
		"internal/temperature_main",
		10,
		std::bind(&FanRegulatorNode::receiveTempMain, this, _1)
	);
	this->tempCPUSubscription = this->create_subscription<std_msgs::msg::Float32>(
		"internal/temperature_cpu",
		10,
		std::bind(&FanRegulatorNode::receiveTempCPU, this, _1)
	);
	this->motorStatusLSubscription = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motors_status",
		10,
		std::bind(&FanRegulatorNode::receiveMotorLStatus, this, _1)
	);
	this->motorStatusRSubscription = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motors_status",
		10,
		std::bind(&FanRegulatorNode::receiveMotorRStatus, this, _1)
	);
	this->fanOutputPublisher = this->create_publisher<std_msgs::msg::Float32>("internal/fan_output", 10);

	this->updateTimer = this->create_wall_timer(period, std::bind(&FanRegulatorNode::update, this));
}

void FanRegulatorNode::update() {
	bool motorWarning = this->motorLTempWarning || this->motorRTempWarning;
	bool tempWarning = this->tempMain > this->thresholdMainWarning || this->tempCPU > thresholdCPUWarning;
	bool tempCritical = this->tempMain > this->thresholdMainCritical || this->tempCPU > thresholdCPUCritical;

	double duty = this->fanLevelLow;
	if (motorWarning || tempCritical) {
		duty = this->fanLevelHigh;
	} else if (tempWarning) {
		duty = this->fanLevelMedium;
	}

	auto message = std_msgs::msg::Float32();
	message.data = duty;
	this->fanOutputPublisher->publish(message);
}

void FanRegulatorNode::receiveTempMain(const std_msgs::msg::Float32::SharedPtr message) {
	this->tempMain = message->data;
}

void FanRegulatorNode::receiveTempCPU(const std_msgs::msg::Float32::SharedPtr message) {
	this->tempCPU = message->data;
}

void FanRegulatorNode::receiveMotorLStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	this->motorLTempWarning = message->thermal_warning;
}

void FanRegulatorNode::receiveMotorRStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	this->motorRTempWarning = message->thermal_warning;
}

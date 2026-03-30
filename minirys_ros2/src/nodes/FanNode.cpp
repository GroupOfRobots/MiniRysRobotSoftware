#include "minirys_ros2/nodes/FanNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

FanNode::FanNode(rclcpp::NodeOptions options):
	Node("fan_ve", options),
	output(0.0f) {
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(2.0));
	this->declare_parameter("pwmFrequency", rclcpp::ParameterValue(10.0));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	auto pwmFrequency = this->get_parameter("pwmFrequency").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pwmFrequency " << pwmFrequency);

	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initializing");
	this->pwm = PWMPin::makeShared(0, 0);
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initialized");
	this->pwm->setFrequency(pwmFrequency);
	this->pwm->setDuty(0.0);
	this->pwm->enable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: set up and enabled");

	this->fanOutputSubscriber = this->create_subscription<std_msgs::msg::Float32>(
		"internal/fan_output",
		10,
		std::bind(&FanNode::receiveFanOutput, this, _1)
	);
	this->updateTimer = this->create_wall_timer(period, std::bind(&FanNode::update, this));
}

FanNode::~FanNode() {
	this->pwm->disable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: disabled");
}

void FanNode::update() {
	// This might throw a std::runtime_error - but we DO want to abort if it does
	this->pwm->setDuty(this->output);
}

void FanNode::receiveFanOutput(const std_msgs::msg::Float32::SharedPtr message) {
	this->output = message->data;
}

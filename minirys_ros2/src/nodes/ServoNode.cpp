#include "minirys_ros2/nodes/ServoNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;


ServoNode::ServoNode(rclcpp::NodeOptions options):
	Node("servo_ve", options),
	output(0.125f) {
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(10.0));
	this->declare_parameter("pwmFrequency", rclcpp::ParameterValue(50.0));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	auto pwmFrequency = this->get_parameter("pwmFrequency").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pwmFrequency " << pwmFrequency);

	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initializing");
	this->pin = 13;
	this->pwm = PWMPin::makeShared(0, 1);
	this->pwm->setFrequency(pwmFrequency);
	this->pwm->setDuty(0.2);
	this->pwm->enable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: set up and enabled");

	this->servoOutputSubscriber = this->create_subscription<std_msgs::msg::Float32>(
		"internal/pwm_servo_output",
		10,
		std::bind(&ServoNode::receiveServoOutput, this, _1)
	);
	this->updateTimer = this->create_wall_timer(period, std::bind(&ServoNode::update, this));
}

ServoNode::~ServoNode() {
	this->setDownPosition();
	std::this_thread::sleep_for(500ms);
	this->pwm->disable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: disabled");
}

void ServoNode::update() {
	// This might throw a std::runtime_error - but we DO want to abort if it does
	this->pwm->setDuty(this->output);
}

void ServoNode::receiveServoOutput(const std_msgs::msg::Float32::SharedPtr message) {
	this->output = message->data;

}

void ServoNode::setUpPosition() {
	this->pwm->setDuty(0.017);
}

void ServoNode::setDownPosition() {
	this->pwm->setDuty(0.125);
}

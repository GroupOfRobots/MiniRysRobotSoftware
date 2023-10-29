#include "minirys_ros2/nodes/ServoNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PWM_CHANNEL 1

ServoNode::ServoNode(rclcpp::NodeOptions options):
	Node("servo_ve", options),
	output(0.015f) {
	// bcm2835_init();
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(2.0));
	this->declare_parameter("pwmFrequency", rclcpp::ParameterValue(50.0));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	auto pwmFrequency = this->get_parameter("pwmFrequency").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pwmFrequency " << pwmFrequency);

	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initializing");
	this->pin = 13;
	// bcm2835_gpio_fsel(13, BCM2835_GPIO_FSEL_ALT0);
	this->pwm = PWMPin::makeShared(0, 1);
	// RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initialized");
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

	// this->setDownPosition();
    // std::this_thread::sleep_for(1s);
	// this->setUpPosition();
}

ServoNode::~ServoNode() {
	this->pwm->disable();
	// bcm2835_close();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: disabled");
}

void ServoNode::update() {
	// This might throw a std::runtime_error - but we DO want to abort if it does
	this->pwm->setDuty(this->output);
	// this->setUpPosition();
	// std::this_thread::sleep_for(1s);
	// this->setUpPosition();
}

void ServoNode::receiveServoOutput(const std_msgs::msg::Float32::SharedPtr message) {
	this->output = message->data;
	// Move servo to down position
	// this->setDownPosition();
    // std::this_thread::sleep_for(1s);
	// this->setUpPosition();

}

void ServoNode::setUpPosition() {
	this->pwm->setDuty(0.134);
}

void ServoNode::setDownPosition() {
	this->pwm->setDuty(0.015);
}

#include "minirys_ros2/nodes/GrabberNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;


GrabberNode::GrabberNode(rclcpp::NodeOptions options):
	Node("servo_ve", options),
	output(0.125f) {
	this->declare_parameter("pwmFrequency", rclcpp::ParameterValue(50.0));

	auto period = std::chrono::duration<double>(0.5);
	auto pwmFrequency = this->get_parameter("pwmFrequency").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pwmFrequency " << pwmFrequency);

	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initializing");
    this->is_closed = false;
	this->pin = 13;
	this->pwm = PWMPin::makeShared(0, 1);
	this->pwm->setFrequency(pwmFrequency);
	this->is_closed = false;
	float PWMperiod = 1000000.0 / pwmFrequency;
	// limity to 700 i 2300
	this->closeDuty = 1150 / PWMperiod;
	RCLCPP_INFO_STREAM(this->get_logger(), "Got closeDuty " << this->closeDuty);
	this->openDuty = 2080 / PWMperiod;
	RCLCPP_INFO_STREAM(this->get_logger(), "Got openDuty " << this->openDuty);

	this->pwm->enable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: set up and enabled");

	this->grabberStateSubscriber = this->create_subscription<std_msgs::msg::Bool>(
		"minirys3/servo_status",
		10,
		std::bind(&GrabberNode::receiveGrabberState, this, _1)
	);

	this->updateTimer = this->create_wall_timer(period, std::bind(&GrabberNode::update, this));
}

GrabberNode::~GrabberNode() {

	this->pwm->setDuty(this->closeDuty);
	std::this_thread::sleep_for(500ms);
	this->pwm->disable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: disabled");
}

void GrabberNode::receiveGrabberState(const std_msgs::msg::Bool::SharedPtr message) {
	this->is_closed = message->data;
}

void GrabberNode::update() {
    if (this->is_closed) {
	    this->pwm->setDuty(this->openDuty);
		RCLCPP_INFO_STREAM(this->get_logger(), "PWM: servo open");
    }
    else {
        this->pwm->setDuty(this->closeDuty);
		RCLCPP_INFO_STREAM(this->get_logger(), "PWM: servo closed");
    }
}


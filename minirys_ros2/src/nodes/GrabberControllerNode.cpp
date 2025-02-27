#include "minirys_ros2/nodes/GrabberControllerNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;


GrabberControllerNode::GrabberControllerNode(rclcpp::NodeOptions options):
	Node("servo_ve", options),
	output(0.125f) {
	// this->declare_parameter("updateFrequency", rclcpp::ParameterValue(10.0));
	this->declare_parameter("pwmFrequency", rclcpp::ParameterValue(50.0));

	auto period = std::chrono::duration<double>(3.0);
	// RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	auto pwmFrequency = this->get_parameter("pwmFrequency").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: pwmFrequency " << pwmFrequency);

	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: initializing");
    this->is_open = false;
	this->pin = 13;
	this->pwm = PWMPin::makeShared(0, 1);
	this->pwm->setFrequency(pwmFrequency);
    // this->min_pulse_width = 700;
	// this->pwm->max_pulse_width = 2300;
	this->is_open = false;
	float PWMperiod = 1000000.0 / pwmFrequency;
	// limity to 700 i 2300
	this->openDuty = 1200 / PWMperiod;
	RCLCPP_INFO_STREAM(this->get_logger(), "Got openDuty " << this->openDuty);
	this->closeDuty = 1500 / PWMperiod;
	RCLCPP_INFO_STREAM(this->get_logger(), "Got closeDuty " << this->closeDuty);

	// this->pwm->setDuty(0.2);
	this->pwm->enable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: set up and enabled");

	this->updateTimer = this->create_wall_timer(period, std::bind(&GrabberControllerNode::update, this));
}

GrabberControllerNode::~GrabberControllerNode() { // wyłączenie
	// this->setOpenPosition();
	
	this->pwm->setDuty(this->openDuty);
	std::this_thread::sleep_for(500ms);
	this->pwm->disable();
	RCLCPP_INFO_STREAM(this->get_logger(), "PWM: disabled");
}

void GrabberControllerNode::update() {
	// otwiaramy jesli byl zamkniety, zamykamy jesli byl otwarty
    if (this->is_open) {
	    this->pwm->setDuty(this->closeDuty); // 0 stopni
        this->is_open = false;
		RCLCPP_INFO_STREAM(this->get_logger(), "PWM: servo closed");
    }
    else {
        this->pwm->setDuty(this->openDuty); // 180 stopni
        this->is_open = true;
		RCLCPP_INFO_STREAM(this->get_logger(), "PWM: servo open");
    }
}


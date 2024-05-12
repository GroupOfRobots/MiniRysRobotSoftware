#include "minirys_ros2/nodes/ServoControllerNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

ServoControllerNode::ServoControllerNode(rclcpp::NodeOptions options):
	Node("servo_controller_cs", options),
	servoStatus(false),
	balanceMode(false) {
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(10.0));
	this->declare_parameter("servoDutyUp", rclcpp::ParameterValue(0.016));
	this->declare_parameter("servoDutyDown", rclcpp::ParameterValue(0.131));


	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	this->servoDutyUp = this->get_parameter("servoDutyUp").as_double();
	this->servoDutyDown = this->get_parameter("servoDutyDown").as_double();


	this->balanceModeSubscriber = this->create_subscription<std_msgs::msg::Bool>(
		"balance_mode",
		10,
		std::bind(&ServoControllerNode::receiveBalanceMode, this, _1)
	);
    this->servoStatusSubscriber = this->create_subscription<std_msgs::msg::Bool>(
		"servo_status",
		10,
		std::bind(&ServoControllerNode::receiveServoStatus, this, _1)
	);
	this->angularPoseSubscriber = this->create_subscription<minirys_msgs::msg::AngularPose>(
		"internal/angular_pose",
		10,
		std::bind(&ServoControllerNode::receiveAngularPose, this, _1)
	);

	this->servoOutputPublisher = this->create_publisher<std_msgs::msg::Float32>("internal/pwm_servo_output", 10);

	this->updateTimer = this->create_wall_timer(period, std::bind(&ServoControllerNode::update, this));
}

void ServoControllerNode::update() {

	double duty = this->servoDutyDown;
	if (this->servoStatus && !this->balanceMode && this->robotAngularPosition < -1.0) {
		duty = this->servoDutyUp;
	} else {
		duty = this->servoDutyDown;
	}

	auto message = std_msgs::msg::Float32();
	message.data = duty;
	this->servoOutputPublisher->publish(message);
}

void ServoControllerNode::receiveBalanceMode(const std_msgs::msg::Bool::SharedPtr message) {
	this->balanceMode = message->data;
}

void ServoControllerNode::receiveAngularPose(const minirys_msgs::msg::AngularPose::SharedPtr message) {
	this->robotAngularPosition = message->angular_position;
}

void ServoControllerNode::receiveServoStatus(const std_msgs::msg::Bool::SharedPtr message) {
	this->servoStatus = message->data;
}

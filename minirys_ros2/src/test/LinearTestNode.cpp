#include "minirys_ros2/test/LinearTestNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

LinearTestNode::LinearTestNode(rclcpp::NodeOptions options):
	Node("linear_test_cs", options),
	linearVelocity(0.0),
	linearDistance(0.0) {
    this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("linearVelocity", rclcpp::ParameterValue(0.1));
	this->declare_parameter("linearDistance", rclcpp::ParameterValue(2.0));

	auto period = std::chrono::duration<double>(2.0 / this->get_parameter("updateFrequency").as_double());
    this->updateFrequency = this->get_parameter("updateFrequency").as_double();
	this->linearVelocity = this->get_parameter("linearVelocity").as_double();
	this->linearDistance = this->get_parameter("linearDistance").as_double();


	this->odometrySubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom",
		10,
		std::bind(&LinearTestNode::receiveOdometry, this, _1)
	);

	this->velocityCommandPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

	// this->updateTimer = this->create_wall_timer(period, std::bind(&ServoControllerNode::update, this));
	this->move_linear();
}

void LinearTestNode::move_linear() {

	double duration = this->linearDistance / this->linearVelocity;
    msg = geometry_msgs::msg::Twist();
    msg.linear.x = this->linearVelocity;

    auto rate = rclcpp::Rate(this->updateFrequency);
    auto now = this->get_clock()->now();
    auto finish = now + rclcpp::Duration::from_seconds(duration);

    while (this->get_clock()->now() < finish)
    {
		this->velocityCommandPublisher->publish(msg);
        rate.sleep();
    } 

	this->velocityCommandPublisher->publish(geometry_msgs::msg::Twist());
}

void LinearTestNode::receiveOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
	this->odometry = message;
}


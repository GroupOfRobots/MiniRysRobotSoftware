#include "minirys_ros2/test/CircleTestNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

CircleTestNode::CircleTestNode(rclcpp::NodeOptions options):
	Node("circle_test", options),
	angularVelocity(0.31416),
	numberOfCircles(1.0) {
    this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("angularVelocity", rclcpp::ParameterValue(0.1 * M_PI));
	this->declare_parameter("numberOfCircles", rclcpp::ParameterValue(1.0));
	this->declare_parameter("turnLeft", rclcpp::ParameterValue(true));

	auto period = std::chrono::duration<double>(2.0 / this->get_parameter("updateFrequency").as_double());
    this->updateFrequency = this->get_parameter("updateFrequency").as_double();
	this->angularVelocity = this->get_parameter("angularVelocity").as_double();
	this->numberOfCircles = this->get_parameter("numberOfCircles").as_double();
	this->turnLeft = this->get_parameter("turnLeft").as_bool();


	this->odometrySubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom",
		10,
		std::bind(&CircleTestNode::receiveOdometry, this, _1)
	);

	this->velocityCommandPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

	// this->updateTimer = this->create_wall_timer(period, std::bind(&ServoControllerNode::update, this));
	this->move_circle();
}

void CircleTestNode::move_circle() {

	double duration = this->numberOfCircles * ((2 * M_PI) / this->angularVelocity);
    msg = geometry_msgs::msg::Twist();
    msg.angular.z = this->angularVelocity * (this->turnLeft ? 1 : -1);

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

void CircleTestNode::receiveOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
	this->odometry = message;
}


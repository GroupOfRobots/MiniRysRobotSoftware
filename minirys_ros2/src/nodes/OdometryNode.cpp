#include "minirys_ros2/nodes/OdometryNode.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

OdometryNode::OdometryNode(rclcpp::NodeOptions options):
	Node("odometry_cs", options),
	poseX(0.0),
	poseY(0.0),
	poseTheta(0.0),
	poseValid(true),
	motorPositionL(0.0),
	motorPositionR(0.0),
	motorPositionLPrev(0.0),
	motorPositionRPrev(0.0),
	motorSpeedL(0.0),
	motorSpeedR(0.0) {
	// Declare parameters
	// General settings
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));

	// Robot geometry
    this->declare_parameter("invertLeftMotor", rclcpp::ParameterValue(false));
    this->declare_parameter("invertRightMotor", rclcpp::ParameterValue(false));
	this->declare_parameter("wheelRadius", rclcpp::ParameterValue(0.0));
	this->declare_parameter("wheelSeparation", rclcpp::ParameterValue(0.0));
	this->declare_parameter("wheelRadiusCorrection", rclcpp::ParameterValue(1.0));
	this->declare_parameter("wheelSeparationCorrection", rclcpp::ParameterValue(1.0));

	// Get parameters
	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
    this->invertLeftMotor = this->get_parameter("invertLeftMotor").as_bool();
    this->invertRightMotor = this->get_parameter("invertRightMotor").as_bool();
	this->wheelRadius = this->get_parameter("wheelRadius").as_double();
	this->wheelSeparation = this->get_parameter("wheelSeparation").as_double();
	this->wheelRadiusCorrection = this->get_parameter("wheelRadiusCorrection").as_double();
	this->wheelSeparationCorrection = this->get_parameter("wheelSeparationCorrection").as_double();

	// Set up subscriptions, publishers, services and timers
	this->motorPositionLSubscription = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_position_l",
		10,
		std::bind(&OdometryNode::receiveMotorPositionL, this, _1)
	);
	this->motorPositionRSubscription = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_position_r",
		10,
		std::bind(&OdometryNode::receiveMotorPositionR, this, _1)
	);
	this->motorSpeedLSubscription = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_speed_l",
		10,
		std::bind(&OdometryNode::receiveMotorSpeedL, this, _1)
	);
	this->motorSpeedRSubscription = this->create_subscription<std_msgs::msg::Float64>(
		"internal/motor_speed_r",
		10,
		std::bind(&OdometryNode::receiveMotorSpeedR, this, _1)
	);
	this->motorStatusLSubscription = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_l",
		10,
		std::bind(&OdometryNode::receiveMotorStatusL, this, _1)
	);
	this->motorStatusRSubscription = this->create_subscription<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_r",
		10,
		std::bind(&OdometryNode::receiveMotorStatusR, this, _1)
	);

	this->odometryValidPublisher = this->create_publisher<std_msgs::msg::Bool>("odom_valid", 10);
	this->odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

	this->setPoseService = this->create_service<minirys_msgs::srv::SetPose>(
		"set_pose",
		std::bind(&OdometryNode::setPose, this, _1, _2)
	);

	this->updateTimer = this->create_wall_timer(period, std::bind(&OdometryNode::update, this));
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void OdometryNode::update() {
	double wheelCircL = this->wheelRadius * (2.0 / (this->wheelRadiusCorrection + 1.0));
	double wheelCircR = this->wheelRadius * (2.0 / (this->wheelRadiusCorrection + 1.0));
	double distanceL = (this->motorPositionLPrev - this->motorPositionL) * wheelCircL;
	double distanceR = (this->motorPositionRPrev - this->motorPositionR) * wheelCircR;
	this->motorPositionLPrev = this->motorPositionL;
	this->motorPositionRPrev = this->motorPositionR;

	double distanceAvg = (distanceL + distanceR) / 2.0;
	double thetaChange = (distanceL - distanceR) / (this->wheelSeparation * this->wheelSeparationCorrection);
	this->poseTheta += thetaChange;
	if (this->poseTheta > M_PI) {
		this->poseTheta -= 2.0 * M_PI;
	} else if (this->poseTheta < -M_PI) {
		this->poseTheta += 2.0 * M_PI;
	}

	double poseChangeX = distanceAvg * cos(this->poseTheta);
	double poseChangeY = distanceAvg * sin(this->poseTheta);

	this->poseX += poseChangeX;
	this->poseY += poseChangeY;

	double speedL = this->motorSpeedL * wheelCircL;
	double speedR = this->motorSpeedR * wheelCircR;

    //first, we'll publish the transform over tf

    auto odom_trans = geometry_msgs::msg::TransformStamped();
    odom_trans.header.stamp = this->get_clock()->now();
    odom_trans.header.frame_id = this->envNamespace + "/odom";
    odom_trans.child_frame_id = this->envNamespace + "/base_footprint";

    odom_trans.transform.translation.x = this->poseX;
    odom_trans.transform.translation.y = this->poseY;
    odom_trans.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, this->poseTheta);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    //RCLCPP_INFO_STREAM(this->get_logger(), "Theta: " << this->poseTheta);

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);

	auto messageOdom = nav_msgs::msg::Odometry();
	messageOdom.header.stamp = this->get_clock()->now();
	messageOdom.header.frame_id = this->envNamespace + "/odom";
    messageOdom.child_frame_id = this->envNamespace + "/base_link";
	messageOdom.pose.pose.position.x = this->poseX;
	messageOdom.pose.pose.position.y = this->poseY;
	messageOdom.pose.pose.orientation.w = cos(this->poseTheta / 2);
	messageOdom.pose.pose.orientation.x = 0;
	messageOdom.pose.pose.orientation.y = 0;
	messageOdom.pose.pose.orientation.z = sin(this->poseTheta / 2);
	messageOdom.twist.twist.linear.x = (speedL + speedR) / 2.0;
	messageOdom.twist.twist.linear.y = 0;
	messageOdom.twist.twist.linear.z = 0;
	messageOdom.twist.twist.angular.x = 0;
	messageOdom.twist.twist.angular.y = 0;
	messageOdom.twist.twist.angular.z = (speedL - speedR) / (this->wheelSeparation * this->wheelSeparationCorrection);
	this->odometryPublisher->publish(messageOdom);

	auto messageOdomValid = std_msgs::msg::Bool();
	messageOdomValid.data = this->poseValid;
	this->odometryValidPublisher->publish(messageOdomValid);
}

void OdometryNode::receiveMotorPositionL(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorPositionL = message->data * (this->invertLeftMotor ? -1 : 1);
}

void OdometryNode::receiveMotorPositionR(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorPositionR = message->data * (this->invertRightMotor ? -1 : 1);
}

void OdometryNode::receiveMotorSpeedL(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorSpeedL = message->data * (this->invertLeftMotor ? -1 : 1);
}

void OdometryNode::receiveMotorSpeedR(const std_msgs::msg::Float64::SharedPtr message) {
	this->motorSpeedR = message->data * (this->invertRightMotor ? -1 : 1);
}

void OdometryNode::receiveMotorStatusL(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	this->poseValid = this->poseValid && !message->step_loss_a && !message->step_loss_b;
}

void OdometryNode::receiveMotorStatusR(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message) {
	this->poseValid = this->poseValid && !message->step_loss_a && !message->step_loss_b;
}

void OdometryNode::setPose(
	const minirys_msgs::srv::SetPose::Request::SharedPtr request,
	minirys_msgs::srv::SetPose::Response::SharedPtr /* response */
) {
	auto reqPosition = request.get()->pose.position;
	this->poseX = reqPosition.x;
	this->poseY = reqPosition.y;

	auto reqOrientation = request.get()->pose.orientation;
	tf2::Quaternion orientQuat(
		reqOrientation.x,
		reqOrientation.y,
		reqOrientation.z,
		reqOrientation.w
	);
	tf2::Matrix3x3 orientMat(orientQuat);
	double roll = NAN;
	double pitch = NAN;
	double yaw = NAN;
	orientMat.getRPY(roll, pitch, yaw);
	this->poseTheta = yaw;

	this->poseValid = true;
}

#include "minirys_ros2/nodes/IMUNode.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

IMUNode::IMUNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options):
	Node("imu_vr", options),
	imu(i2cBus),
	angleFilteredPrev(0.0),
	updatePeriod(0.0) {
	RCLCPP_INFO_STREAM(this->get_logger(), "LSM6DS3: initializing");
	this->imu.initialize();
	RCLCPP_INFO_STREAM(this->get_logger(), "LSM6DS3: initialized");

	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("filterFactor", rclcpp::ParameterValue(0.05));
	this->declare_parameter("angleCorrection", rclcpp::ParameterValue(0.089545));
	this->declare_parameter("angleHistorySize", rclcpp::ParameterValue(4));
    this->declare_parameter("gyroOffsetX", rclcpp::ParameterValue(3.850143));
    this->declare_parameter("gyroOffsetY", rclcpp::ParameterValue(-5.236362));
    this->declare_parameter("gyroOffsetZ", rclcpp::ParameterValue(-4.604892));

	this->updatePeriod = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << this->updatePeriod.count());
	this->filterFactor = this->get_parameter("filterFactor").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: filterFactor " << this->filterFactor);
	this->angleCorrection = this->get_parameter("angleCorrection").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: angleCorrection " << this->angleCorrection);
	this->angleHistorySize = this->get_parameter("angleHistorySize").as_int();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: angleHistorySize " << this->angleHistorySize);

	this->imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("internal/imu", 10);
	this->angularPosePublisher = this->create_publisher<minirys_msgs::msg::AngularPose>("internal/angular_pose", 10);

	this->updateTimer = this->create_wall_timer(updatePeriod, std::bind(&IMUNode::update, this));

    angle_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void IMUNode::update() {
	// Read the clock and the raw IMU data
	auto now = this->get_clock()->now();
	double gyroX = this->imu.readFloatGyroX() - this->get_parameter("gyroOffsetX").as_double();
	double gyroY = this->imu.readFloatGyroY() - this->get_parameter("gyroOffsetY").as_double();
	double gyroZ = this->imu.readFloatGyroZ() - this->get_parameter("gyroOffsetZ").as_double();
	double accelX = this->imu.readFloatAccelX();
	double accelY = this->imu.readFloatAccelY();
	double accelZ = this->imu.readFloatAccelZ();

	// Calculate the raw roll angle, push it into the history
	double angleRaw = std::atan2(accelX, std::sqrt(accelY * accelY + accelZ * accelZ)) - this->angleCorrection;
	this->angleRawHistory.push_front(angleRaw);
	while (this->angleRawHistory.size() > this->angleHistorySize) {
		this->angleRawHistory.pop_back();
	}

	// Calculate the average of last N angles
	double angleSum = std::accumulate(
		this->angleRawHistory.begin(),
		this->angleRawHistory.end(),
		0.0
	);
	double angleAvg = angleSum / this->angleRawHistory.size();

	// Calculate the filtered angle and save it
	double periodInS = this->updatePeriod.count() / 1000.0;
	double periodInHz = 1000.0 / this->updatePeriod.count();
	double angleUpdated = this->angleFilteredPrev + gyroY * periodInS;
	double angleFiltered = this->filterFactor * angleAvg + angleUpdated * (1.0 - this->filterFactor);
	double angularVelocity = (angleFiltered - this->angleFilteredPrev) * periodInHz;
	this->angleFilteredPrev = angleFiltered;

	// Prepare the messages
	auto imuMessage = sensor_msgs::msg::Imu();
	imuMessage.header.frame_id = "imu";
	imuMessage.header.stamp = now;
	imuMessage.angular_velocity.x = gyroX;
	imuMessage.angular_velocity.y = gyroY;
	imuMessage.angular_velocity.z = gyroZ;
	imuMessage.linear_acceleration.x = accelX;
	imuMessage.linear_acceleration.y = accelY;
	imuMessage.linear_acceleration.z = accelZ;

	auto poseMessage = minirys_msgs::msg::AngularPose();
	poseMessage.header.frame_id = "imu";
	poseMessage.header.stamp = now;
	poseMessage.angular_position = angleFiltered;
	poseMessage.angular_velocity = angularVelocity;

    // Transform broadcaster
    auto angle_trans = geometry_msgs::msg::TransformStamped();
    angle_trans.header.stamp = this->get_clock()->now();
    angle_trans.header.frame_id = "base_link";
    angle_trans.child_frame_id = "robot_base";

    angle_trans.transform.translation.x = 0.0;
    angle_trans.transform.translation.y = 0.0;
    angle_trans.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, -(angleFiltered + 1.5708), 0); //robot model was defined in horizontal position -> +1.5708 and imu is physically mounted in different orientation -> -()
    angle_trans.transform.rotation.x = q.x();
    angle_trans.transform.rotation.y = q.y();
    angle_trans.transform.rotation.z = q.z();
    angle_trans.transform.rotation.w = q.w();

    //send the transform
    angle_broadcaster->sendTransform(angle_trans);

	// Send the messages
	this->imuPublisher->publish(imuMessage);
	this->angularPosePublisher->publish(poseMessage);
}

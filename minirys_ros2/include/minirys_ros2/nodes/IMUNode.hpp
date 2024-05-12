#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/angular_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <cstdlib>

#include <I2CBus.hpp>
#include <LSM6.hpp>

class IMUNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(IMUNode);

	IMUNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options);

	~IMUNode() override = default;

private:
	LSM6 imu;

	std::list<double> angleRawHistory;
	double angleFilteredPrev;

	std::string envNamespace = std::getenv("NAMESPACE");
	std::chrono::duration<double> updatePeriod;
	double filterFactor;
	double angleCorrection;
	unsigned int angleHistorySize;

    std::shared_ptr<tf2_ros::TransformBroadcaster> angle_broadcaster;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

	rclcpp::Publisher<minirys_msgs::msg::AngularPose>::SharedPtr angularPosePublisher;

	void update();
};

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/angular_pose.hpp>
#include <sensor_msgs/msg/imu.hpp>

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

	std::chrono::duration<double> updatePeriod;
	double filterFactor;
	double angleCorrection;
	unsigned int angleHistorySize;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

	rclcpp::Publisher<minirys_msgs::msg::AngularPose>::SharedPtr angularPosePublisher;

	void update();
};

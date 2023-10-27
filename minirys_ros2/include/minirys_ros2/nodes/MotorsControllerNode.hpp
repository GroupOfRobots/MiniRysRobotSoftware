#pragma once

#include "minirys_ros2/helpers/PIDRegulator.hpp"
#include "minirys_ros2/helpers/TimeMeasure.hpp"
#include "minirys_ros2/nodes/pid.cpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <minirys_msgs/msg/angular_pose.hpp>
#include <minirys_msgs/msg/motor_command.hpp>
#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class MotorsControllerNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(MotorsControllerNode);

	explicit MotorsControllerNode(rclcpp::NodeOptions options);

	~MotorsControllerNode() override;

private:
	bool enabledL;

    bool enabledR;

	bool balancing;

	bool targetBalancing;

	bool targetBalancingPrev;

	double targetForwardSpeed;

	double targetRotationSpeed;

	double robotAngularPosition;

	double robotAngularVelocity;

	double motorSpeedL;

	double motorSpeedR;

	int standingUpDir;

	int standingUpPhase;

	rclcpp::Time standingUpStart;

	bool enableSpeedRegulator;

	bool invertLeftMotor;

	bool invertRightMotor;

	// m/s?, TODO: get from maxWheelSpeed
	double maxLinearSpeed;
	// rps, TODO: get from maxWheelSpeed
	double maxRotationSpeed;
	// rps
	double maxWheelSpeed;
	// rad?
    double maxStandUpSpeed;
    // rad?
	double maxBalancingAngle;

	PID anglePid = PID(0.5f);
	PID speedPid = PID(0.5f);

	PIDRegulator angleRegulator;

	PIDRegulator speedRegulator;

	rclcpp::Clock steadyROSClock;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<minirys_msgs::msg::MotorCommand>::SharedPtr motorCommandPublisher;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityCommandSubscriber;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr balanceModeSubscriber;

	rclcpp::Subscription<minirys_msgs::msg::AngularPose>::SharedPtr angularPoseSubscriber;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorSpeedLSubscriber;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motorSpeedRSubscriber;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusLSubscriber;

	rclcpp::Subscription<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusRSubscriber;

	uint64_t spinCount;
	TimeMeasure timeMeasure;

	void update();

	void receiveVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);

	void receiveBalanceMode(const std_msgs::msg::Bool::SharedPtr message);

	void receiveAngularPose(const minirys_msgs::msg::AngularPose::SharedPtr message);

	void receiveMotorLSpeed(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorRSpeed(const std_msgs::msg::Float64::SharedPtr message);

	void receiveMotorLStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);

	void receiveMotorRStatus(const minirys_msgs::msg::MotorDriverStatus::SharedPtr message);

	std::pair<double, double> calculateSpeedsFlat() const;

	std::pair<double, double> calculateSpeedsBalancing();

	std::pair<double, double> standUp();
};

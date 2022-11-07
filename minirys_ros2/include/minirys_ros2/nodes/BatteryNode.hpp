#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/battery_status.hpp>

#include <I2CBus.hpp>
#include <MAX1161.hpp>

class BatteryNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(BatteryNode);

	BatteryNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options);

	~BatteryNode() override = default;

private:
	static constexpr double DEFAULT_CELL_VOLTAGE = 3.7;

	double movingAverageWeight;
	double undervoltageThresholdWarning;
	double undervoltageThresholdError;
	double cellMultiplier1;
	double cellMultiplier2;
	double cellMultiplier3;

	MAX1161 adc;

	double voltageAcc[3];

	bool undervoltageWarning;

	bool undervoltageError;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<minirys_msgs::msg::BatteryStatus>::SharedPtr batteryStatusPublisher;

	void update();
};

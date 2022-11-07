#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <GPIOPin.hpp>
#include <I2CBus.hpp>
#include <VL53L1X.hpp>

class DistanceNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(DistanceNode);

	explicit DistanceNode(rclcpp::NodeOptions options);

	~DistanceNode() override;

private:
	I2CBus::SharedPtr i2c3;

	I2CBus::SharedPtr i2c5;

	GPIOPin::SharedPtr gpio6;

	GPIOPin::SharedPtr gpio16;

	GPIOPin::SharedPtr gpio19;

	GPIOPin::SharedPtr gpio20;

	GPIOPin::SharedPtr gpio21;

	GPIOPin::SharedPtr gpio26;

	VL53L1X::SharedPtr sensors[6];

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distancePublishers[6];

	void update();
};

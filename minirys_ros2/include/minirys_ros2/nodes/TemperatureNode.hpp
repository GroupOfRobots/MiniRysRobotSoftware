#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <I2CBus.hpp>
#include <MCP9808.hpp>

#include <mutex>
#include <string>

class TemperatureNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(TemperatureNode);

	explicit TemperatureNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options);

	~TemperatureNode() override;

private:
	MCP9808 tempSensor;

	std::string cpuTempPath;

	std::mutex cpuFileMutex;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mainTempPublisher;

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpuTempPublisher;

	void update();

	int readCPUTemp();
};

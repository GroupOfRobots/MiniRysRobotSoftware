#include "minirys_ros2/nodes/TemperatureNode.hpp"

#include <chrono>
#include <fstream>
#include <functional>

using namespace std::chrono_literals;

TemperatureNode::TemperatureNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options):
	Node("temperature_vr", options),
	tempSensor(i2cBus) {
	RCLCPP_INFO_STREAM(this->get_logger(), "Temperature sensor: initializing");
	this->tempSensor.initialize();
	this->tempSensor.wakeup();
	RCLCPP_INFO_STREAM(this->get_logger(), "Temperature sensor: initialized");

	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(2.0));
	this->declare_parameter("cpuTemperaturePath", rclcpp::ParameterValue("/sys/class/thermal/thermal_zone0/temp"));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	this->cpuTempPath = this->get_parameter("cpuTemperaturePath").as_string();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: cpuTempPath " << this->cpuTempPath);

	this->mainTempPublisher = this->create_publisher<std_msgs::msg::Float32>("internal/temperature_main", 10);
	this->cpuTempPublisher = this->create_publisher<std_msgs::msg::Float32>("internal/temperature_cpu", 10);

	this->updateTimer = this->create_wall_timer(period, std::bind(&TemperatureNode::update, this));
}

TemperatureNode::~TemperatureNode() {
	RCLCPP_INFO_STREAM(this->get_logger(), "Temperature sensor: shutting down");
	this->tempSensor.shutdown();
}

void TemperatureNode::update() {
	auto mainTempMessage = std_msgs::msg::Float32();
	try {
		mainTempMessage.data = this->tempSensor.readTemperature();
	} catch (const std::runtime_error& e) {
		mainTempMessage.data = 9999;
		RCLCPP_WARN_STREAM(this->get_logger(), "Error reading temperature from the sensor: " << e.what());
	}
	auto cpuTempMessage = std_msgs::msg::Float32();
	cpuTempMessage.data = static_cast<float>(this->readCPUTemp()) / 1000.0f;

	this->mainTempPublisher->publish(mainTempMessage);
	this->cpuTempPublisher->publish(cpuTempMessage);
}

int TemperatureNode::readCPUTemp() {
	std::lock_guard<std::mutex> guard(this->cpuFileMutex);

	std::ifstream file;
	file.open(this->cpuTempPath.c_str(), std::ofstream::in);
	if (!file.is_open() || !file.good()) {
		file.close();
		RCLCPP_WARN_STREAM(this->get_logger(), "Error opening CPU temperature file " << this->cpuTempPath);
		return 9999999;
	}
	std::string fileValue;
	file >> fileValue;
	file.close();

	try {
		return std::stoi(fileValue);
	} catch (const std::invalid_argument& e) {
		RCLCPP_WARN_STREAM(this->get_logger(), "Error parsing CPU temperature: " << e.what());
		return 9999998;
	} catch (const std::out_of_range& e) {
		RCLCPP_WARN_STREAM(this->get_logger(), "Error parsing CPU temperature: " << e.what());
		return 9999997;
	}
}

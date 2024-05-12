#include "minirys_ros2/nodes/BatteryNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

BatteryNode::BatteryNode(I2CBus::SharedPtr i2cBus, rclcpp::NodeOptions options):
	Node("battery_vr", options),
	adc(i2cBus, MAX1161::MAX11613),
	voltageAcc{DEFAULT_CELL_VOLTAGE, DEFAULT_CELL_VOLTAGE, DEFAULT_CELL_VOLTAGE},
	undervoltageWarning(false),
	undervoltageError(false) {
	RCLCPP_INFO_STREAM(this->get_logger(), "Battery ADC: initializing");
	this->adc.initialize();
	RCLCPP_INFO_STREAM(this->get_logger(), "Battery ADC: initialized");

	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(10.0));
	this->declare_parameter("movingAverageWeight", rclcpp::ParameterValue(0.5));
	this->declare_parameter("undervoltageThresholdWarning", rclcpp::ParameterValue(3.3));
	this->declare_parameter("undervoltageThresholdError", rclcpp::ParameterValue(3.1));
	this->declare_parameter("cellMultiplier1", rclcpp::ParameterValue(2.55));
	this->declare_parameter("cellMultiplier2", rclcpp::ParameterValue(5.11));
	this->declare_parameter("cellMultiplier3", rclcpp::ParameterValue(7.68));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	this->movingAverageWeight = this->get_parameter("movingAverageWeight").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: movingAverageWeight " << this->movingAverageWeight);
	// *INDENT-OFF*
	// Uncrustify really DOESN'T LIKE rclcpp's logging macros
	this->undervoltageThresholdWarning = this->get_parameter("undervoltageThresholdWarning").as_double();
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Got param: undervoltageThresholdWarning " << this->undervoltageThresholdWarning
	);
	this->undervoltageThresholdError = this->get_parameter("undervoltageThresholdError").as_double();
	RCLCPP_INFO_STREAM(
		this->get_logger(),
		"Got param: undervoltageThresholdError " << this->undervoltageThresholdError
	);
	// *INDENT-ON*
	this->cellMultiplier1 = this->get_parameter("cellMultiplier1").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: cellMultiplier1 " << this->cellMultiplier1);
	this->cellMultiplier2 = this->get_parameter("cellMultiplier2").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: cellMultiplier2 " << this->cellMultiplier2);
	this->cellMultiplier3 = this->get_parameter("cellMultiplier3").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: cellMultiplier3 " << this->cellMultiplier3);

	this->batteryStatusPublisher = this->create_publisher<minirys_msgs::msg::BatteryStatus>(
		"internal/battery_status",
		10
	);
	this->updateTimer = this->create_wall_timer(period, std::bind(&BatteryNode::update, this));
}

void BatteryNode::update() {
	double cell1 = this->adc.readChannel(0) * this->cellMultiplier1;
	double cell2 = this->adc.readChannel(0) * this->cellMultiplier2 - cell1;
	double cell3 = this->adc.readChannel(0) * this->cellMultiplier3 - cell1 - cell2;

	this->voltageAcc[0] = this->voltageAcc[0] * (1.0f - this->movingAverageWeight) + cell1 * this->movingAverageWeight;
	this->voltageAcc[1] = this->voltageAcc[1] * (1.0f - this->movingAverageWeight) + cell2 * this->movingAverageWeight;
	this->voltageAcc[2] = this->voltageAcc[2] * (1.0f - this->movingAverageWeight) + cell3 * this->movingAverageWeight;

	double uvWarning = this->undervoltageThresholdWarning;
	double uvError = this->undervoltageThresholdError;

	this->undervoltageWarning = this->undervoltageWarning || std::any_of(
		std::begin(this->voltageAcc),
		std::end(this->voltageAcc),
		[uvWarning](double v) {
			return v < uvWarning;
		}
	);
	this->undervoltageError = this->undervoltageError || std::any_of(
		std::begin(this->voltageAcc),
		std::end(this->voltageAcc),
		[uvError](double v) {
			return v < uvError;
		}
	);
	auto message = minirys_msgs::msg::BatteryStatus();
	message.header.frame_id = "battery";
	message.header.stamp = this->get_clock()->now();
	message.voltage_cell1 = static_cast<float>(this->voltageAcc[0]);
	message.voltage_cell2 = static_cast<float>(this->voltageAcc[1]);
	message.voltage_cell3 = static_cast<float>(this->voltageAcc[2]);
	message.undervoltage_warning = this->undervoltageWarning;
	message.undervoltage_error = this->undervoltageError;
	this->batteryStatusPublisher->publish(message);
}

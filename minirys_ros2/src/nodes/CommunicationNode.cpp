#include "minirys_ros2/nodes/CommunicationNode.hpp"

#include <functional>
#include <set>
#include <sstream>
#include <boost/process.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

CommunicationNode::CommunicationNode(rclcpp::NodeOptions options):
	Node("communication_cs", options) {
	RCLCPP_INFO(this->get_logger(), "Initializing data relays");

	// Battery
	this->batteryStatusSubscription = this->create_subscription<minirys_msgs::msg::BatteryStatus>(
		"internal/battery_status",
		10,
		std::bind(&CommunicationNode::receiveBatteryStatus, this, _1)
	);
	this->batteryStatePublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);

	// Distances
	for (int i = 0; i < 6; i++) {
		std::string inTopicName = "internal/distance_" + std::to_string(i);
		std::string outTopicName = "distance_" + std::to_string(i);
		this->distanceSubscriptions[i] = this->create_subscription<sensor_msgs::msg::Range>(
			inTopicName,
			10,
			std::bind(&CommunicationNode::receiveDistance, this, _1)
		);
		this->distancePublishers[i] = this->create_publisher<sensor_msgs::msg::Range>(outTopicName, 10);
	}

	// Temperatures
	this->mainTempSubscription = this->create_subscription<std_msgs::msg::Float32>(
		"internal/temperature_main",
		10,
		std::bind(&CommunicationNode::receiveMainTemperature, this, _1)
	);
	this->mainTempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("temperature_main", 10);
	this->cpuTempSubscription = this->create_subscription<std_msgs::msg::Float32>(
		"internal/temperature_cpu",
		10,
		std::bind(&CommunicationNode::receiveCPUTemperature, this, _1)
	);
	this->cpuTempPublisher = this->create_publisher<sensor_msgs::msg::Temperature>("temperature_cpu", 10);

	// IMU
	this->imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
		"internal/imu",
		10,
		std::bind(&CommunicationNode::receiveIMU, this, _1)
	);
	this->imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

	this->robotsNamespacesPublisher = this->create_publisher<minirys_msgs::msg::RobotsNamespaces>("robots_namespaces", 10);

	timer_ = this->create_wall_timer(1s, std::bind(&CommunicationNode::publishNamespaces, this));


	RCLCPP_INFO(this->get_logger(), "Data relays initialized");

	this->nodeName_ = this->get_name();
	this->nodeNamespace_ = this->get_namespace();

	if (!this->nodeNamespace_.empty() && this->nodeNamespace_[0] == '/') {
        this->nodeNamespace_.erase(0, 1);
    }

}

void CommunicationNode::receiveBatteryStatus(const minirys_msgs::msg::BatteryStatus::SharedPtr msgIn) {
	sensor_msgs::msg::BatteryState msgOut;
	msgOut.header = msgIn->header;

	msgOut.voltage = msgIn->voltage_cell1 + msgIn->voltage_cell2 + msgIn->voltage_cell3;
	msgOut.temperature = NAN;
	msgOut.current = NAN;
	msgOut.charge = NAN;
	msgOut.capacity = 2.6;
	msgOut.design_capacity = 2.6;
	msgOut.percentage = (msgOut.voltage - 3.3) / 0.9;
	msgOut.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
	msgOut.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
	msgOut.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
	msgOut.present = true;
	msgOut.cell_voltage.resize(3);
	msgOut.cell_voltage[0] = msgIn->voltage_cell1;
	msgOut.cell_voltage[1] = msgIn->voltage_cell2;
	msgOut.cell_voltage[2] = msgIn->voltage_cell3;
	msgOut.cell_temperature.resize(3);
	msgOut.cell_temperature[0] = NAN;
	msgOut.cell_temperature[1] = NAN;
	msgOut.cell_temperature[2] = NAN;
	msgOut.location = "";
	msgOut.serial_number = "";

	this->batteryStatePublisher->publish(msgOut);
}

void CommunicationNode::receiveDistance(const sensor_msgs::msg::Range::SharedPtr msgIn) {
	std::string indexStr = msgIn->header.frame_id.substr(9, 1);
	int index = std::stoi(indexStr);
	this->distancePublishers[index]->publish(*msgIn);
}

void CommunicationNode::receiveMainTemperature(const std_msgs::msg::Float32::SharedPtr msgIn) {
	sensor_msgs::msg::Temperature msgOut;
	msgOut.header.stamp = this->now();
	msgOut.header.frame_id = "temperature_main";

	msgOut.temperature = msgIn->data;
	msgOut.variance = 0;
	this->mainTempPublisher->publish(msgOut);
}

void CommunicationNode::receiveCPUTemperature(const std_msgs::msg::Float32::SharedPtr msgIn) {
	sensor_msgs::msg::Temperature msgOut;
	msgOut.header.stamp = this->now();
	msgOut.header.frame_id = "temperature_cpu";

	msgOut.temperature = msgIn->data;
	msgOut.variance = 0;
	this->cpuTempPublisher->publish(msgOut);
}

void CommunicationNode::receiveIMU(const sensor_msgs::msg::Imu::SharedPtr msgIn) {
	this->imuPublisher->publish(*msgIn);
}

std::vector<std::string> CommunicationNode::getNamespacesWithNode() {
	auto nodes_names = this->get_node_names();
	std::set<std::string> namespaces;
	for (auto &name : nodes_names)
	{
		std::size_t pos = name.find('/' + this->nodeName_);
		if (pos != std::string::npos) {
            std::string namespace_ = name.substr(0, pos);
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_.erase(0, 1);
            }
			if (namespace_ != this->nodeNamespace_){
				namespaces.insert(namespace_);
			}
		}
	}
    
    return std::vector<std::string>(namespaces.begin(), namespaces.end());
}

void CommunicationNode::publishNamespaces() {
    std::vector<std::string> namespaces = getNamespacesWithNode();
	auto message = minirys_msgs::msg::RobotsNamespaces();
    message.namespaces = namespaces;
    this->robotsNamespacesPublisher->publish(message);
}
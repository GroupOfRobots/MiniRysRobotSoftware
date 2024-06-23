#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/battery_status.hpp>
#include <minirys_msgs/msg/robots_namespaces.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>


class CommunicationNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(CommunicationNode);

	explicit CommunicationNode(rclcpp::NodeOptions options);

private:
	std::string nodeName_;
	std::string nodeNamespace_;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<minirys_msgs::msg::BatteryStatus>::SharedPtr batteryStatusSubscription;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryStatePublisher;

	rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr distanceSubscriptions[6];
	rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr distancePublishers[6];

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mainTempSubscription;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr mainTempPublisher;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cpuTempSubscription;
	rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr cpuTempPublisher;

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;

	rclcpp::Publisher<minirys_msgs::msg::RobotsNamespaces>::SharedPtr robotsNamespacesPublisher;

	void receiveBatteryStatus(const minirys_msgs::msg::BatteryStatus::SharedPtr msgIn);

	void receiveDistance(const sensor_msgs::msg::Range::SharedPtr msgIn);

	void receiveMainTemperature(const std_msgs::msg::Float32::SharedPtr msgIn);

	void receiveCPUTemperature(const std_msgs::msg::Float32::SharedPtr msgIn);

	void receiveIMU(const sensor_msgs::msg::Imu::SharedPtr msgIn);

	std::vector<std::string> getNamespacesWithNode();

	void publishNamespaces();
};

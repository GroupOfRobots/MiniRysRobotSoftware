#pragma once

#include "minirys_ros2/helpers/RTTExecutor.hpp"

#include <minirys_msgs/msg/rt_test_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rttest/rttest.h>

class RTTestNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(RTTestNode);

	RTTestNode(rclcpp::NodeOptions options, RTTExecutor::SharedPtr executor);

	~RTTestNode() override;

	void init();

private:
	// Parameter values
	bool setScheduling;
	bool setMemoryStrategy;
	bool lockMemory;
	int schedulerPriority;
	size_t schedulerPolicy;
	size_t iterations;
	timespec updatePeriod;
	size_t stackSize;
	uint64_t prefaultDynamicSize;
	char* filename;

	RTTExecutor::SharedPtr executor;

	rclcpp::memory_strategy::MemoryStrategy::SharedPtr memoryStrategy;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<minirys_msgs::msg::RTTestResult>::SharedPtr resultPublisher;

	void update();
};

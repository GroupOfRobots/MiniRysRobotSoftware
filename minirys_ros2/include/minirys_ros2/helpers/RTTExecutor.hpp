// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/macros.hpp>
#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rmw/rmw.h>
#include <rttest/rttest.h>
#include <rttest/utils.hpp>
#include <tlsf_cpp/tlsf.hpp>

#include <minirys_msgs/msg/rt_test_result.hpp>

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sched.h>
#include <vector>

/**
 * Instrumented executor that syncs Executor::spin functions with rttest_spin
 *
 * Based on the rtt_executor from the pendulum_demo package:
 * https://github.com/ros2/demos/blob/master/pendulum_control/include/pendulum_control/rtt_executor.hpp
 */
class RTTExecutor: public rclcpp::Executor {
public:
	// IDK why cppcheck fails this, but it does
	// cppcheck-suppress unknownMacro
	RCLCPP_DISABLE_COPY(RTTExecutor);

	RCLCPP_SMART_PTR_DEFINITIONS(RTTExecutor);

	/**
	 * Constructor
	 *
	 * Extends default Executor constructor
	 */
	RTTExecutor(const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions());

	int init(int iterations, char* filename, bool setupMemory);

	/**
	 * Return true if the executor is currently spinning.
	 *
	 * @return True if rclcpp is running and if the "running" boolean is set to true.
	 */
	bool isRunning() const {
		return rclcpp::ok() && this->running;
	}

	minirys_msgs::msg::RTTestResult getRTTestResults() const;

	/**
	 * Wrap executor::spin into rttest_spin.
	 *
	 * Do all the work available to the executor for as many iterations specified by rttest.
	 */
	void spin() override;

	void saveResults(const rttest_results& results, int64_t sampleTime);

	/**
	 * Core component of the executor - do a little bit of work and update extra state.
	 *
	 * @param arg Anonymous argument, will be casted as a pointer to an RTTExecutor.
	 */
	static void* loopCallback(void* arg);

	static int setupRT(int cpuCore, int schedulerPriority, size_t schedulerPolicy);

private:
	rclcpp::Logger logger;

	// True if the executor is spinning.
	bool running;

	// True if rttest has initialized and hasn't been stopped yet.
	bool rttestReady;

	// Absolute timestamp at which the first data point was collected in rttest.
	timespec startTime;

	// For storing accumulated performance statistics.
	rttest_results results;
	bool resultsAvailable;

	int64_t lastSample;
};

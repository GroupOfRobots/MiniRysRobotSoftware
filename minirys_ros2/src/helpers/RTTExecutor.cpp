#include "minirys_ros2/helpers/RTTExecutor.hpp"

#include <iostream>

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using TLSFAllocatorMemoryStrategy = AllocatorMemoryStrategy<tlsf_heap_allocator<void>>;

RTTExecutor::RTTExecutor(const rclcpp::ExecutorOptions& options):
	rclcpp::Executor(options),
	logger(rclcpp::get_logger("RTTExecutor")),
	running(false),
	rttestReady(false),
	startTime{},
	results{},
	resultsAvailable(false),
	lastSample(0) {
	// Fill the rttest readiness flag
	this->rttestReady = rttest_running() != 0;
}

int RTTExecutor::init(int iterations, char* filename, bool setupMemory = false) {
	timespec updatePeriod {};
	updatePeriod.tv_nsec = 10000000;

	RCLCPP_INFO(this->logger, "Initializing RTTest");
	int initResult = rttest_init(
		iterations,
		updatePeriod,
		// Scheduler policy
		SCHED_RR,
		// Scheduler priority
		98,
		// Stack size, 1MB
		1ULL * 1024ULL * 1024,
		// Prefault dynamic size, 32MB
		32ULL * 1024ULL * 1024ULL,
		filename
	);
	if (initResult != 0) {
		RCLCPP_WARN(this->logger, "RTTest initialization failure");
	}

	if (!setupMemory) {
		return initResult;
	}

	RCLCPP_INFO(this->logger, "Setting executor memory strategy");
	this->set_memory_strategy(TLSFAllocatorMemoryStrategy::make_shared());

	RCLCPP_INFO(this->logger, "Locking memory");
	/**
	 * Lock the memory:
	 * - Lock currently paged memory (using mlockall),
	 * - Prefault the stack,
	 * - Commit a pool of dynamic memory.
	 */
	int lockMemoryResult = rttest_lock_memory();
	if (lockMemoryResult != 0) {
		RCLCPP_WARN(this->logger, "Error locking the memory");
	}
	int prefaultStackResult = rttest_prefault_stack();
	if (prefaultStackResult != 0) {
		RCLCPP_WARN(this->logger, "Error prefaulting the stack");
	}
	int lockDynamicResult = rttest_lock_and_prefault_dynamic();
	if (lockDynamicResult != 0) {
		RCLCPP_WARN(this->logger, "Error locking and prefaulting dynamic memory");
	}

	return initResult + lockMemoryResult + prefaultStackResult + lockDynamicResult;
}

minirys_msgs::msg::RTTestResult RTTExecutor::getRTTestResults() const {
	minirys_msgs::msg::RTTestResult msg {};
	if (!this->resultsAvailable) {
		return msg;
	}

	timespec currentTime {};
	clock_gettime(CLOCK_MONOTONIC, &currentTime);

	msg.stamp.sec = currentTime.tv_sec;
	msg.stamp.nanosec = currentTime.tv_nsec;
	msg.current_latency = this->lastSample;
	msg.mean_latency = this->results.mean_latency;
	msg.min_latency = this->results.min_latency;
	msg.max_latency = this->results.max_latency;
	msg.minor_pagefaults = this->results.minor_pagefaults;
	msg.major_pagefaults = this->results.major_pagefaults;

	return msg;
}

void RTTExecutor::spin() {
	clock_gettime(CLOCK_MONOTONIC, &this->startTime);

	// This call will block until rttest is finished,
	// calling loopCallback at periodic intervals specified on the command line.
	// Note: I'd love to use std::bind here, but rttest is written in C :(
	rttest_spin(RTTExecutor::loopCallback, static_cast<void*>(this));

	// Clean up state and write results after rttest has finished spinning.
	this->running = false;
	rttest_write_results();

	if (rttest_running() != 0) {
		rttest_finish();
	}

	this->rttestReady = rttest_running() != 0;
}

void RTTExecutor::saveResults(const rttest_results& results, int64_t sampleTime) {
	this->results = results;
	this->lastSample = sampleTime;
	this->resultsAvailable = true;
	this->running = true;
}

void* RTTExecutor::loopCallback(void* arg) {
	// Cast the argument so that we can access the executor's state.
	auto* executor = static_cast<RTTExecutor*>(arg);

	// If the input pointer was NULL or invalid, or if rclcpp has stopped, signal rttest to stop.
	if (executor == nullptr || !rclcpp::ok()) {
		rttest_finish();
		return nullptr;
	}

	// Single-threaded spin_some: do as much work as we have available.
	executor->spin_some();

	// Retrieve rttest statistics accumulated so far and store them in the executor.
	rttest_results results {};
	int64_t sampleTime = 0;
	if (rttest_get_statistics(&results) != 0) {
		return nullptr;
	}
	rttest_get_sample_at(results.iteration, &sampleTime);

	executor->saveResults(results, sampleTime);

	return nullptr;
}

int RTTExecutor::setupRT(int cpuCore, int schedulerPriority, size_t schedulerPolicy) {
	int setAffinityResult = 0;
	if (cpuCore >= 0) {
		cpu_set_t mask;
		CPU_ZERO(&mask);
		CPU_SET(cpuCore, &mask);
		setAffinityResult = sched_setaffinity(0, sizeof(mask), &mask);
		if (setAffinityResult != 0) {
			std::cerr << "Error setting CPU affinity" << std::endl;
		}
	}

	int setSchedulerResult = 0;
	if (schedulerPriority > 0) {
		sched_param param;
		param.sched_priority = schedulerPriority;
		setSchedulerResult = sched_setscheduler(0, schedulerPolicy, &param);
		if (setSchedulerResult != 0) {
			std::cerr << "Error setting scheduler priority and policy" << std::endl;
		}
	}

	return setAffinityResult + setSchedulerResult;
}

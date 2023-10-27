#include "minirys_ros2/nodes/RTTestNode.hpp"

#include <chrono>
#include <functional>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <tlsf_cpp/tlsf.hpp>

using namespace std::chrono_literals;

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
using TLSFAllocatorMemoryStrategy = AllocatorMemoryStrategy<tlsf_heap_allocator<void>>;

RTTestNode::RTTestNode(rclcpp::NodeOptions options, RTTExecutor::SharedPtr executor):
	Node("minirys_rttest", options),
	schedulerPolicy(SCHED_OTHER),
	updatePeriod{},
	filename(nullptr),
	executor(std::move(executor)) {
	this->memoryStrategy = TLSFAllocatorMemoryStrategy::make_shared();

	this->setScheduling = this->declare_parameter("setScheduling", false);
	this->setMemoryStrategy = this->declare_parameter("setMemoryStrategy", false);
	this->lockMemory = this->declare_parameter("lockMemory", false);

	this->schedulerPriority = this->declare_parameter("schedulerPriority", 80);
	auto schedulerPolicyStr = this->declare_parameter("schedulerPolicy", "other");
	if (schedulerPolicyStr == "fifo") {
		this->schedulerPolicy = SCHED_FIFO;
	} else if (schedulerPolicyStr == "rr") {
		this->schedulerPolicy = SCHED_RR;
	}

	this->iterations = this->declare_parameter("iterations", 0);
	this->updatePeriod.tv_sec = 0;
	this->updatePeriod.tv_nsec = this->declare_parameter("updatePeriod", 10000000);

	this->stackSize = this->declare_parameter("stackSizeMB", 10) * 1024 * 1024;
	this->prefaultDynamicSize = this->declare_parameter("prefaultDynamicSizeMB", 1024) * 1024ULL * 1024ULL;

	auto filenameStr = this->declare_parameter("filename", "");
	if (filenameStr.length() != 0) {
		// NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
		this->filename = new char [filenameStr.length() + 1];
		strcpy(this->filename, filenameStr.c_str());
	}

	// Periodic publication of result messages - publisher + timer
	this->resultPublisher = this->create_publisher<minirys_msgs::msg::RTTestResult>("rttest_results", 10);
	this->updateTimer = this->create_wall_timer(1s, std::bind(&RTTestNode::update, this));
}

RTTestNode::~RTTestNode() {
	delete [] filename;
}

void RTTestNode::init() {
	RCLCPP_INFO(this->get_logger(), "Initializing RT test node");

	// Set executor's memory strategy
	if (this->setMemoryStrategy) {
		RCLCPP_INFO(this->get_logger(), "Setting memory strategy");
		this->executor->set_memory_strategy(this->memoryStrategy);
	}

	RCLCPP_INFO(
		this->get_logger(),
		"Initializing rttest with iterations: %ld, periodNs: %ld, schedPolicy %ld, schedPrio %d, stackSize %ld, prefDynSize %lu, filename %s",
		this->iterations,
		this->updatePeriod.tv_nsec,
		this->schedulerPolicy,
		this->schedulerPriority,
		this->stackSize,
		this->prefaultDynamicSize,
		this->filename
	);
	auto initResult = rttest_init(
		this->iterations,
		this->updatePeriod,
		this->schedulerPolicy,
		this->schedulerPriority,
		this->stackSize,
		this->prefaultDynamicSize,
		this->filename
	);
	if (initResult != 0) {
		RCLCPP_WARN(this->get_logger(), "RTTest initialization failure");
	}

	if (this->setScheduling) {
		RCLCPP_INFO(this->get_logger(), "Setting scheduling parameters");
		if (rttest_set_thread_default_priority() != 0) {
			RCLCPP_WARN(
				this->get_logger(),
				"Couldn't set the scheduling parameters. Verify /etc/security/limits.conf is set properly."
			);
		}
	}

	// Lock the memory
	if (this->get_parameter("lockMemory").as_bool()) {
		RCLCPP_INFO(this->get_logger(), "Locking memory");
		/**
		 * Lock the memory:
		 * - Lock currently paged memory (using mlockall),
		 * - Prefault the stack,
		 * - Commit a pool of dynamic memory.
		 */
		int lockResult = rttest_lock_memory();
		lockResult += rttest_prefault_stack();
		lockResult += rttest_lock_and_prefault_dynamic();
		if (lockResult != 0) {
			RCLCPP_WARN(
				this->get_logger(),
				"Couldn't lock the memory - insufficient resources or permissions. "
				"Pagefaults from reading pages not yet mapped into RAM will be recorded."
			);
		}
	}

	RCLCPP_INFO(this->get_logger(), "Initialization done!");
}

void RTTestNode::update() {
	auto msg = this->executor->getRTTestResults();
	if (msg.max_latency == 0) {
		return;
	}
	this->resultPublisher->publish(msg);
}

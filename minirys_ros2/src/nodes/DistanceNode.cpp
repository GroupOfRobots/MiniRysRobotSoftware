#include "minirys_ros2/nodes/DistanceNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

DistanceNode::DistanceNode(rclcpp::NodeOptions options):
	Node("distance_vr", options) {
	RCLCPP_INFO_STREAM(this->get_logger(), "I2C: initializing");
	this->i2c3 = I2CBus::makeShared("/dev/i2c-3");
	this->i2c5 = I2CBus::makeShared("/dev/i2c-5");
	RCLCPP_INFO_STREAM(this->get_logger(), "I2C: initialized");

	RCLCPP_INFO_STREAM(this->get_logger(), "GPIO: initializing");
	this->gpio6 = GPIOPin::makeShared("/sys/class/gpio/gpio6");
	this->gpio16 = GPIOPin::makeShared("/sys/class/gpio/gpio16");
	this->gpio19 = GPIOPin::makeShared("/sys/class/gpio/gpio19");
	this->gpio20 = GPIOPin::makeShared("/sys/class/gpio/gpio20");
	this->gpio21 = GPIOPin::makeShared("/sys/class/gpio/gpio21");
	this->gpio26 = GPIOPin::makeShared("/sys/class/gpio/gpio26");
	RCLCPP_INFO_STREAM(this->get_logger(), "GPIO: initializing");

	RCLCPP_INFO_STREAM(this->get_logger(), "VL53L1X: initializing");
	this->sensors[0] = VL53L1X::makeShared(this->i2c3, this->gpio6, 0x29, 100ms);
	this->sensors[1] = VL53L1X::makeShared(this->i2c3, this->gpio16, 0x29, 100ms);
	this->sensors[2] = VL53L1X::makeShared(this->i2c3, this->gpio19, 0x29, 100ms);
	this->sensors[3] = VL53L1X::makeShared(this->i2c5, this->gpio20, 0x29, 100ms);
	this->sensors[4] = VL53L1X::makeShared(this->i2c5, this->gpio21, 0x29, 100ms);
	this->sensors[5] = VL53L1X::makeShared(this->i2c5, this->gpio26, 0x29, 100ms);

	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(20.0));
	/// TODO: decide whether ranging parameters (distance mode, timing budget, timeouts) should be parametrized

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());

	for (int i = 0; i < 6; i++) {
		std::string topicName = "internal/distance_" + std::to_string(i);
		this->distancePublishers[i] = this->create_publisher<sensor_msgs::msg::Range>(topicName, 10);

		this->sensors[i]->powerOff();
		
	}
    this->scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("internal/laser_scan", 10);

	for (int i = 0; i < 6; i++) {
		this->sensors[i]->powerOn();
		this->sensors[i]->setAddress(0x29 + i + 1);
		
		this->sensors[i]->initialize();
		
		this->sensors[i]->setDistanceMode(VL53L1X::DISTANCE_MODE_SHORT);
		//this->sensors[i]->setDistanceMode(VL53L1X::DISTANCE_MODE_LONG);
		//this->sensors[i]->setTimingBudget(VL53L1X::TIMING_BUDGET_50_MS);
		this->sensors[i]->setTimingBudget(VL53L1X::TIMING_BUDGET_20_MS);
		this->sensors[i]->setInterMeasurementPeriod(25);

		this->sensors[i]->startRanging();
		
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "VL53L1X: initialized");

	this->updateTimer = this->create_wall_timer(period, std::bind(&DistanceNode::update, this));
}

DistanceNode::~DistanceNode() {
	RCLCPP_INFO_STREAM(this->get_logger(), "VL53L1X: stopping");
	for (auto& sensor: this->sensors) {
		sensor->stopRanging();
		sensor->powerOff();
	}
	RCLCPP_INFO_STREAM(this->get_logger(), "VL53L1X: powered off");
}

void DistanceNode::update() {
	auto message = sensor_msgs::msg::Range();
	message.min_range = 0;
	message.max_range = 4;
	message.field_of_view = 0.4712389;
	message.radiation_type = sensor_msgs::msg::Range::INFRARED;

    auto scan = sensor_msgs::msg::LaserScan();
    scan.angle_min = 0;
    scan.angle_max = 6.28318530718;
    scan.angle_increment = 1.57079632679;
    scan.time_increment = 0.1;
    scan.scan_time = 0.1;
    scan.range_min = 0;
    scan.range_max = 4;

	for (int i = 0; i < 6; i++) {
		auto distance = this->sensors[i]->getDistance();
		if (distance == 65535) {
			RCLCPP_WARN_STREAM(this->get_logger(), "VL53L1X: timeout on sensor " << i);
		}
		message.header.stamp = this->get_clock()->now();
		message.header.frame_id = "distance_" + std::to_string(i);
		message.range = static_cast<float>(distance) / 1000.0f;
		this->distancePublishers[i]->publish(message);

	}

//     Scan message for tof version
//
//     auto distance0 = this->sensors[0]->getDistance();
//     auto distance1 = this->sensors[1]->getDistance();
//     auto distance2 = this->sensors[2]->getDistance();
//     auto distance5 = this->sensors[5]->getDistance();
//     auto range0 = (static_cast<float>(distance0) / 1000.0f) + 0.033f; //adding offsets to ranges in order to make laser scan in centre of robot
//     auto range1 = (static_cast<float>(distance1) / 1000.0f) + 0.082f;
//     auto range2 = (static_cast<float>(distance2) / 1000.0f) + 0.05f;
//     auto range5 = (static_cast<float>(distance5) / 1000.0f) + 0.05f;
//     std::vector<float> scan_ranges = {range0, range2, range1, range5};
//     scan.ranges = scan_ranges;
//     scan.header.stamp = this->get_clock()->now();
//     scan.header.frame_id = "laser_scan";
//     this->scanPublisher->publish(scan);
}

#include "minirys_ros2/nodes/MotorsNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

MotorsNode::MotorsNode(rclcpp::NodeOptions options):
	Node("motors_ve", options),
	speedL(0.0f),
	speedR(0.0f) {
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("stepsPerRevolution", rclcpp::ParameterValue(200.0));
	this->declare_parameter("maxSpeed", rclcpp::ParameterValue(2.0 * M_PI));
	this->declare_parameter("acceleration", rclcpp::ParameterValue(2.0 * M_PI));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	this->stepsPerRevolution = this->get_parameter("stepsPerRevolution").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: stepsPerRevolution " << this->stepsPerRevolution);
	this->maxSpeed = this->get_parameter("maxSpeed").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: maxSpeed " << this->maxSpeed);
	// Max speed in steps per second
	auto maxSpeedSPS = this->maxSpeed / 2.0 / M_PI * this->stepsPerRevolution;
	auto fullSpeedSPS = maxSpeedSPS * 10.0;
	auto acceleration = this->get_parameter("acceleration").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: acceleration " << acceleration);
	// Acceleration in steps per second
	auto accelerationSPS = acceleration / 2.0 / M_PI * this->stepsPerRevolution;

	RCLCPP_INFO_STREAM(this->get_logger(), "SPI: initializing");
	this->spi = SPIBus::makeShared("/dev/spidev0.0", SPIBus::SPIBUS_MODE_3, 5000000);
	std::this_thread::sleep_for(100ms);
	RCLCPP_INFO_STREAM(this->get_logger(), "SPI: initialized");

	RCLCPP_INFO_STREAM(this->get_logger(), "GPIO: initializing");
	this->resetPin = GPIOPin::makeShared("/sys/class/gpio/gpio22");
	this->busyPin = GPIOPin::makeShared("/sys/class/gpio/gpio27");
	RCLCPP_INFO_STREAM(this->get_logger(), "GPIO: initialized");

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: initializing");
	this->motors = L6470::makeShared(2, this->spi, this->resetPin);
	std::this_thread::sleep_for(100ms);

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: resetting");
	this->motors->resetDevice();
	std::this_thread::sleep_for(100ms);

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: configuring");
	// Set 62.5kHz PWM frequency
	this->motors->setOscillatorMode(L6470::OSC_MODE_INT_16MHZ_OSCOUT_16MHZ);
	this->motors->setPWMFrequency(L6470::PWM_DIV_1, L6470::PWM_MULT_2);
	// Other CONFIG register values
	this->motors->setSlewRate(L6470::SLEW_RATE_260V_US);
	this->motors->setOCShutdown(true);
	this->motors->setVoltageCompensation(false);
	this->motors->setSwitchMode(L6470::SWITCH_MODE_USER);
	// Setup stepping, speeds etc
	this->motors->configStepMode(L6470::STEP_MODE_32);
	this->motors->setMaxSpeed(maxSpeedSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: MaxSpeed " << maxSpeedSPS);
	this->motors->setMinSpeed(0);
	this->motors->setFullSpeed(fullSpeedSPS);
    //this->motors->setFullSpeed(602.7);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: FullSpeed " << fullSpeedSPS);
	this->motors->setAcceleration(accelerationSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Acceleration " << accelerationSPS);
	this->motors->setDeceleration(accelerationSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Deceleration " << accelerationSPS);
	// Current/voltage settings
	this->motors->setOCThreshold(3000);
	this->motors->setAccelerationKVAL(0x64);  //80
	this->motors->setDecelerationKVAL(0x64);  //80
	this->motors->setRunKVAL(0x80);  //B4
	this->motors->setHoldKVAL(0x32);  //00
	// Disable BEMF compensation and the FLAG (alarm) pin
	this->motors->setParam(L6470::REG_ADDR_ST_SLP, 0x00);
	this->motors->setParam(L6470::REG_ADDR_FN_SLP_ACC, 0x00);
	this->motors->setParam(L6470::REG_ADDR_FN_SLP_DEC, 0x00);
	this->motors->setParam(L6470::REG_ADDR_ALARM_EN, 0x00);
	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: setup done");

	this->motorCommandSubscription = this->create_subscription<minirys_msgs::msg::MotorCommand>(
		"internal/motor_command",
		10,
		std::bind(&MotorsNode::receiveMotorCommand, this, _1)
	);

	this->motorPositionLPublisher = this->create_publisher<std_msgs::msg::Float64>("internal/motor_position_l", 10);
	this->motorPositionRPublisher = this->create_publisher<std_msgs::msg::Float64>("internal/motor_position_r", 10);

	this->motorSpeedLPublisher = this->create_publisher<std_msgs::msg::Float64>("internal/motor_speed_l", 10);
	this->motorSpeedRPublisher = this->create_publisher<std_msgs::msg::Float64>("internal/motor_speed_r", 10);

	this->motorStatusLPublisher = this->create_publisher<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_l",
		10
	);
	this->motorStatusRPublisher = this->create_publisher<minirys_msgs::msg::MotorDriverStatus>(
		"internal/motor_status_r",
		10
	);

	this->updateTimer = this->create_wall_timer(period, std::bind(&MotorsNode::update, this));
}

MotorsNode::~MotorsNode() {
	this->motors->softStop();
	this->motors->resetDevice();
	this->resetPin->unset();
}

void MotorsNode::update() {
	std::vector<float> speeds;
	std::vector<L6470::Direction> dirs;
	// requested speeds in steps per second
	auto speedLSPS = this->speedL / (2.0 * M_PI) * this->stepsPerRevolution;
	auto speedRSPS = this->speedR / (2.0 * M_PI) * this->stepsPerRevolution;
	speeds.emplace_back(std::abs(speedLSPS));
	speeds.emplace_back(std::abs(speedRSPS));
	dirs.emplace_back(this->speedL >= 0.0 ? L6470::DIRECTION_FWD : L6470::DIRECTION_REV);
	dirs.emplace_back(this->speedR >= 0.0 ? L6470::DIRECTION_FWD : L6470::DIRECTION_REV);
	this->motors->run(speeds, dirs);

	auto motorPositions = this->motors->getPosition();
	auto motorSpeeds = this->motors->getSpeed();
	auto motorStatuses = this->motors->getStatus();

	auto positionMessageL = std_msgs::msg::Float64();
	auto positionMessageR = std_msgs::msg::Float64();
	auto speedMessageL = std_msgs::msg::Float64();
	auto speedMessageR = std_msgs::msg::Float64();
	auto statusMessageL = minirys_msgs::msg::MotorDriverStatus();
	auto statusMessageR = minirys_msgs::msg::MotorDriverStatus();

	positionMessageL.data = static_cast<double>(motorPositions[0]) * 2.0 * M_PI / (this->stepsPerRevolution * 32.0);
	positionMessageR.data = static_cast<double>(motorPositions[1]) * 2.0 * M_PI / (this->stepsPerRevolution * 32.0);
	speedMessageL.data = static_cast<double>(motorSpeeds[0]) * 2.0 * M_PI / this->stepsPerRevolution * (motorStatuses[0].direction == 0 ? 1 : -1);
	speedMessageR.data = static_cast<double>(motorSpeeds[1]) * 2.0 * M_PI / this->stepsPerRevolution * (motorStatuses[1].direction == 0 ? 1 : -1);
	statusMessageL.hi_z = motorStatuses[0].hiZ;
	statusMessageL.busy = motorStatuses[0].busy;
	statusMessageL.direction = motorStatuses[0].direction;
	statusMessageL.motor_stopped = motorStatuses[0].motorStatus == L6470::MOTOR_STATUS_STOPPED;
	statusMessageL.motor_accelerating = motorStatuses[0].motorStatus == L6470::MOTOR_STATUS_ACCELERATION;
	statusMessageL.motor_decelerating = motorStatuses[0].motorStatus == L6470::MOTOR_STATUS_DECELERATION;
	statusMessageL.motor_const_speed = motorStatuses[0].motorStatus == L6470::MOTOR_STATUS_CONST_SPD;
	statusMessageL.undervoltage = motorStatuses[0].undervoltageLockout;
	statusMessageL.thermal_warning = motorStatuses[0].thermalWarning;
	statusMessageL.thermal_shutdown = motorStatuses[0].thermalShutdown;
	statusMessageL.overcurrent = motorStatuses[0].overcurrent;
	statusMessageL.step_loss_a = motorStatuses[0].stepLossA;
	statusMessageL.step_loss_b = motorStatuses[0].stepLossB;
	statusMessageR.hi_z = motorStatuses[1].hiZ;
	statusMessageR.busy = motorStatuses[1].busy;
	statusMessageR.direction = motorStatuses[1].direction;
	statusMessageR.motor_stopped = motorStatuses[1].motorStatus == L6470::MOTOR_STATUS_STOPPED;
	statusMessageR.motor_accelerating = motorStatuses[1].motorStatus == L6470::MOTOR_STATUS_ACCELERATION;
	statusMessageR.motor_decelerating = motorStatuses[1].motorStatus == L6470::MOTOR_STATUS_DECELERATION;
	statusMessageR.motor_const_speed = motorStatuses[1].motorStatus == L6470::MOTOR_STATUS_CONST_SPD;
	statusMessageR.undervoltage = motorStatuses[1].undervoltageLockout;
	statusMessageR.thermal_warning = motorStatuses[1].thermalWarning;
	statusMessageR.thermal_shutdown = motorStatuses[1].thermalShutdown;
	statusMessageR.overcurrent = motorStatuses[1].overcurrent;
	statusMessageR.step_loss_a = motorStatuses[1].stepLossA;
	statusMessageR.step_loss_b = motorStatuses[1].stepLossB;

	statusMessageL.header.frame_id = "motor_l";
	statusMessageR.header.frame_id = "motor_r";
	statusMessageL.header.stamp = this->get_clock()->now();
	statusMessageR.header.stamp = statusMessageL.header.stamp;

	this->motorPositionLPublisher->publish(positionMessageL);
	this->motorPositionRPublisher->publish(positionMessageR);
	this->motorSpeedLPublisher->publish(speedMessageL);
	this->motorSpeedRPublisher->publish(speedMessageR);
	this->motorStatusLPublisher->publish(statusMessageL);
	this->motorStatusRPublisher->publish(statusMessageR);
}

void MotorsNode::receiveMotorCommand(const minirys_msgs::msg::MotorCommand::SharedPtr message) {
	// Clamp the requested speeds
	this->speedL = std::min(std::max(message->speed_l, -this->maxSpeed), this->maxSpeed);
	this->speedR = std::min(std::max(message->speed_r, -this->maxSpeed), this->maxSpeed);
}

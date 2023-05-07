#include "minirys_ros2/nodes/MotorsNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

MotorsNode::MotorsNode(rclcpp::NodeOptions options):
	Node("motors_ve", options),
	speedL(0.0f),
	speedR(0.0f){
	this->declare_parameter("updateFrequency", rclcpp::ParameterValue(100.0));
	this->declare_parameter("stepsPerRevolution", rclcpp::ParameterValue(200.0));
	this->declare_parameter("maxSpeed", rclcpp::ParameterValue(2.0 * M_PI));
	this->declare_parameter("acceleration", rclcpp::ParameterValue(2.0 * M_PI));
    this->declare_parameter("wheelRadius", rclcpp::ParameterValue(0.0));

	auto period = std::chrono::duration<double>(1.0 / this->get_parameter("updateFrequency").as_double());
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: update period (s) " << period.count());
	this->stepsPerRevolution = this->get_parameter("stepsPerRevolution").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: stepsPerRevolution " << this->stepsPerRevolution);
	this->maxSpeed = this->get_parameter("maxSpeed").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: maxSpeed " << this->maxSpeed);
	// Max speed in steps per second
	auto maxSpeedSPS = this->maxSpeed / 2.0 / M_PI * this->stepsPerRevolution;
	auto fullSpeedSPS = maxSpeedSPS * 0.5;
	auto acceleration = this->get_parameter("acceleration").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: acceleration " << acceleration);
	// Acceleration in steps per second
	auto accelerationSPS = acceleration / 2.0 / M_PI * this->stepsPerRevolution;

    this->wheelRadius = this->get_parameter("wheelRadius").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: wheel radius " << this->wheelRadius);

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: initializing");
    this->motors = std::make_shared<Motors>(BCM2835_SPI_CS0, GPIO_RESET_OUT);

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: resetting");
	this->motors->resetDevice();
	std::this_thread::sleep_for(10ms);

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: configuring");
	// Set 62.5kHz PWM frequency
    this->motors->setOscillatorMode(L6470_CONFIG_OSC_EXT_16MHZ_XTAL_DRIVE, LEFT_MOTOR); //OSCIN pin of right stepper motor driver is physically connected to OSCOUT left motor driver
    this->motors->setOscillatorMode(L6470_CONFIG_OSC_INT_16MHZ_OSCOUT_16MHZ, RIGHT_MOTOR); // in order to synchronize clocks
	this->motors->setPWMFrequency(L6470_CONFIG_PWM_INT_DIV_1, L6470_CONFIG_PWM_DEC_MUL_2);
	// Other CONFIG register values
	this->motors->setPowSlewRate(L6470_CONFIG_POW_SR_260V_us);
	this->motors->setOverCurrentShutdown(L6470_CONFIG_OC_SD_DISABLE);
	this->motors->setVoltageCompensation(TL6470_CONFIG_VS_COMP_DISABLE);
	this->motors->setSwitchModeConfig(TL6470_CONFIG_SW_MODE_USER);
	// Setup stepping, speeds etc
	this->motors->configStepSelMode(L6470_STEP_SEL_1_64);
	this->motors->setMaximumSpeed(maxSpeedSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: MaxSpeed " << maxSpeedSPS);
	this->motors->setMinimumSpeed(0);
	this->motors->setFullStepModeSpeed(fullSpeedSPS);
    //this->motors->setFullSpeed(602.7);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: FullSpeed " << fullSpeedSPS);
	this->motors->setAcceleration(accelerationSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Acceleration " << accelerationSPS);
	this->motors->setDeceleration(accelerationSPS);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Deceleration " << accelerationSPS);
	// Current/voltage settings
	this->motors->setOverCurrentThreshold(L6470_OCD_TH_3000mA);
    //this->motors->setStallThreshold(0x40);
	this->motors->setAccCurrentKVAL(64);  //80/96
	this->motors->setDecCurrentKVAL(0x64);  //80/96
	this->motors->setRunCurrentKVAL(0x64);  //B4 70/96
	this->motors->setHoldCurrentKVAL(0x32);  //40/32
	// Disable BEMF compensation and the FLAG (alarm) pin
	this->motors->setBackEMF();
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

    this->jointPublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	this->updateTimer = this->create_wall_timer(period, std::bind(&MotorsNode::update, this));
}

MotorsNode::~MotorsNode() {
    this->motors->stop();
	this->motors->resetDevice();
}

void MotorsNode::update() {
	std::vector<float> speeds;
	auto speedLSPS = this->speedL / (2.0 * M_PI) * this->stepsPerRevolution;
	auto speedRSPS = this->speedR / (2.0 * M_PI) * this->stepsPerRevolution;
	speeds.emplace_back(speedLSPS);
	speeds.emplace_back(speedRSPS);
    this->motors->setSpeeds(speeds);

    auto motorPositions = this->motors->getPosition();
    auto motorSpeeds = this->motors->getSpeeds();
	auto motorStatuses = this->motors->getMotorStatus();

	auto positionMessageL = std_msgs::msg::Float64();
	auto positionMessageR = std_msgs::msg::Float64();
	auto speedMessageL = std_msgs::msg::Float64();
	auto speedMessageR = std_msgs::msg::Float64();
	auto statusMessageL = minirys_msgs::msg::MotorDriverStatus();
	auto statusMessageR = minirys_msgs::msg::MotorDriverStatus();

    auto step_mode = this->motors->getMicroStepMode();
    auto wheelsJointsPosition = sensor_msgs::msg::JointState();

	positionMessageL.data = static_cast<double>(motorPositions[LEFT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * pow(2, static_cast<int>(step_mode[LEFT_MOTOR])));
	positionMessageR.data = static_cast<double>(motorPositions[RIGHT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * pow(2, static_cast<int>(step_mode[RIGHT_MOTOR])));
	speedMessageL.data = static_cast<double>(motorSpeeds[LEFT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * (motorStatuses[LEFT_MOTOR].direction == 0 ? 1 : -1));
	speedMessageR.data = static_cast<double>(motorSpeeds[RIGHT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * (motorStatuses[RIGHT_MOTOR].direction == 0 ? 1 : -1));
    statusMessageL.hi_z = motorStatuses[LEFT_MOTOR].hiZ;
    statusMessageL.busy = motorStatuses[LEFT_MOTOR].busy;
    statusMessageL.direction = motorStatuses[LEFT_MOTOR].direction;
    statusMessageL.motor_stopped = motorStatuses[LEFT_MOTOR].motorStopped;
    statusMessageL.motor_accelerating = motorStatuses[LEFT_MOTOR].motorAcceleration;
    statusMessageL.motor_decelerating = motorStatuses[LEFT_MOTOR].motorDeceleration;
    statusMessageL.motor_const_speed = motorStatuses[LEFT_MOTOR].motorConstSpeed;
    statusMessageL.undervoltage = motorStatuses[LEFT_MOTOR].underVoltageLockout;
    statusMessageL.thermal_warning = motorStatuses[LEFT_MOTOR].thermalWarning;
    statusMessageL.thermal_shutdown = motorStatuses[LEFT_MOTOR].thermalShutdown;
    statusMessageL.overcurrent = motorStatuses[LEFT_MOTOR].overCurrent;
    statusMessageL.step_loss_a = motorStatuses[LEFT_MOTOR].stepLossA;
    statusMessageL.step_loss_b = motorStatuses[LEFT_MOTOR].stepLossB;
    statusMessageR.hi_z = motorStatuses[RIGHT_MOTOR].hiZ;
    statusMessageR.busy = motorStatuses[RIGHT_MOTOR].busy;
    statusMessageR.direction = motorStatuses[RIGHT_MOTOR].direction;
    statusMessageR.motor_stopped = motorStatuses[RIGHT_MOTOR].motorStopped;
    statusMessageR.motor_accelerating = motorStatuses[RIGHT_MOTOR].motorAcceleration;
    statusMessageR.motor_decelerating = motorStatuses[RIGHT_MOTOR].motorDeceleration;
    statusMessageR.motor_const_speed = motorStatuses[RIGHT_MOTOR].motorConstSpeed;
    statusMessageR.undervoltage = motorStatuses[RIGHT_MOTOR].underVoltageLockout;
    statusMessageR.thermal_warning = motorStatuses[RIGHT_MOTOR].thermalWarning;
    statusMessageR.thermal_shutdown = motorStatuses[RIGHT_MOTOR].thermalShutdown;
    statusMessageR.overcurrent = motorStatuses[RIGHT_MOTOR].overCurrent;
    statusMessageR.step_loss_a = motorStatuses[RIGHT_MOTOR].stepLossA;
    statusMessageR.step_loss_b = motorStatuses[RIGHT_MOTOR].stepLossB;

	statusMessageL.header.frame_id = "motor_l";
	statusMessageR.header.frame_id = "motor_r";
	statusMessageL.header.stamp = this->get_clock()->now();
	statusMessageR.header.stamp = statusMessageL.header.stamp;

    wheelsJointsPosition.header.stamp = this->get_clock()->now();
    wheelsJointsPosition.name.resize(2);
    wheelsJointsPosition.position.resize(2);
    wheelsJointsPosition.name[0] ="base_link_to_rightwheel_joint";
    wheelsJointsPosition.position[0] = static_cast<double>(motorPositions[RIGHT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * pow(2, static_cast<int>(step_mode[RIGHT_MOTOR])));
    wheelsJointsPosition.name[1] ="base_link_to_leftwheel_joint";
    wheelsJointsPosition.position[1] = static_cast<double>(motorPositions[LEFT_MOTOR]) * 2.0 * M_PI / (this->stepsPerRevolution * pow(2, static_cast<int>(step_mode[LEFT_MOTOR])));
    this->jointPublisher->publish(wheelsJointsPosition);

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

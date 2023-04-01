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
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: accelerationSPS " << accelerationSPS);

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: initializing");
    this->motors = std::make_shared<Motors>(BCM2835_SPI_CS0, GPIO_RESET_OUT);

	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: resetting");
	this->motors->resetDevice();
	std::this_thread::sleep_for(100ms);

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
	this->motors->setAccCurrentKVAL(0x80);  //80
	this->motors->setDecCurrentKVAL(0x80);  //80
	this->motors->setRunCurrentKVAL(0x70);  //B4 70
	this->motors->setHoldCurrentKVAL(0x40);  //40
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

	this->updateTimer = this->create_wall_timer(period, std::bind(&MotorsNode::update, this));
}

MotorsNode::~MotorsNode() {
    this->motors->stop();
	this->motors->resetDevice();
}

void MotorsNode::update() {
	std::array<float,2> speeds;
    //RCLCPP_INFO_STREAM(this->get_logger(), "Received motors speed: " << this->speedL << ' ' << this->speedR << '\n');
	auto speedLSPS = this->speedL / (2.0 * M_PI) * this->stepsPerRevolution;
	auto speedRSPS = this->speedR / (2.0 * M_PI) * this->stepsPerRevolution;
    //RCLCPP_INFO_STREAM(this->get_logger(), "Calculated motors speed: " << speedLSPS << ' ' << speedRSPS << '\n');
	speeds[LEFT_MOTOR]=speedLSPS;
	speeds[RIGHT_MOTOR]=speedRSPS;
    //RCLCPP_INFO_STREAM(this->get_logger(), "Set motors speed: " << speeds[0] << ' ' << speeds[1] << '\n');
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

	positionMessageL.data = static_cast<double>(motorPositions[0]) * 2.0 * M_PI / (this->stepsPerRevolution * 32.0);
	positionMessageR.data = static_cast<double>(motorPositions[1]) * 2.0 * M_PI / (this->stepsPerRevolution * 32.0);
	speedMessageL.data = static_cast<double>(motorSpeeds[0]) * 2.0 * M_PI / this->stepsPerRevolution; //* (motorStatuses[0].direction == 0 ? 1 : -1);
	speedMessageR.data = static_cast<double>(motorSpeeds[1]) * 2.0 * M_PI / this->stepsPerRevolution; //* (motorStatuses[1].direction == 0 ? 1 : -1);
    statusMessageL.hi_z = motorStatuses[0].hiZ;
    statusMessageL.busy = motorStatuses[0].busy;
    statusMessageL.direction = motorStatuses[0].direction;
    statusMessageL.motor_stopped = motorStatuses[0].motorStopped;
    statusMessageL.motor_accelerating = motorStatuses[0].motorAcceleration;
    statusMessageL.motor_decelerating = motorStatuses[0].motorDeceleration;
    statusMessageL.motor_const_speed = motorStatuses[0].motorConstSpeed;
    statusMessageL.undervoltage = motorStatuses[0].underVoltageLockout;
    statusMessageL.thermal_warning = motorStatuses[0].thermalWarning;
    statusMessageL.thermal_shutdown = motorStatuses[0].thermalShutdown;
    statusMessageL.overcurrent = motorStatuses[0].overCurrent;
    statusMessageL.step_loss_a = motorStatuses[0].stepLossA;
    statusMessageL.step_loss_b = motorStatuses[0].stepLossB;
    statusMessageR.hi_z = motorStatuses[1].hiZ;
    statusMessageR.busy = motorStatuses[1].busy;
    statusMessageR.direction = motorStatuses[1].direction;
    statusMessageR.motor_stopped = motorStatuses[1].motorStopped;
    statusMessageR.motor_accelerating = motorStatuses[1].motorAcceleration;
    statusMessageR.motor_decelerating = motorStatuses[1].motorDeceleration;
    statusMessageR.motor_const_speed = motorStatuses[1].motorConstSpeed;
    statusMessageR.undervoltage = motorStatuses[1].underVoltageLockout;
    statusMessageR.thermal_warning = motorStatuses[1].thermalWarning;
    statusMessageR.thermal_shutdown = motorStatuses[1].thermalShutdown;
    statusMessageR.overcurrent = motorStatuses[1].overCurrent;
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

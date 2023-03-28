#include "minirys_ros2/nodes/MotorsNode.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

MotorsNode::MotorsNode(rclcpp::NodeOptions options):
	Node("motors_ve", options),
	speedL(0.0f),
	speedR(0.0f),
    motorsEnabled(true),
    motorSpeedLeft(0.0f),
    motorSpeedRight(0.0f),
    maxAcceleration(0.0f),
    invertLeftSpeed(false),
    invertRightSpeed(false){
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
	//auto maxSpeedSPS = this->maxSpeed / 2.0 / M_PI * this->stepsPerRevolution;
	//auto fullSpeedSPS = maxSpeedSPS * 0.5;
	auto acceleration = this->get_parameter("acceleration").as_double();
	RCLCPP_INFO_STREAM(this->get_logger(), "Got param: acceleration " << acceleration);
	// Acceleration in steps per second
	auto accelerationSPS = acceleration / 2.0 / M_PI * this->stepsPerRevolution;
    this->maxAcceleration = accelerationSPS;
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: accelerationSPS " << accelerationSPS);

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: initializing");
    this->motors = std::make_shared<Motors>(BCM2835_SPI_CS0, GPIO_RESET_OUT);

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: configuring");
    this->motors->setUp();

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: resetting");
    this->motors->resetPosition();

    RCLCPP_INFO_STREAM(this->get_logger(), "L6470: setup done");
	/*RCLCPP_INFO_STREAM(this->get_logger(), "SPI: initializing");
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
    this->motors->setOscillatorMode(L6470::OSC_MODE_EXT_16MHZ_XTAL_DRIVE,0); //OSCIN pin of right stepper motor driver is physically connected to OSCOUT left motor driver
    this->motors->setOscillatorMode(L6470::OSC_MODE_INT_16MHZ_OSCOUT_16MHZ,1); // in order to synchronize clocks
    //this->motors->setOscillatorMode(L6470::OSC_MODE_INT_16MHZ_OSCOUT_16MHZ); //old setting without clocks connection
	this->motors->setPWMFrequency(L6470::PWM_DIV_1, L6470::PWM_MULT_2);
	// Other CONFIG register values
	this->motors->setSlewRate(L6470::SLEW_RATE_260V_US);
	this->motors->setOCShutdown(true);
	this->motors->setVoltageCompensation(false);
	this->motors->setSwitchMode(L6470::SWITCH_MODE_USER);
	// Setup stepping, speeds etc
	this->motors->configStepMode(L6470::STEP_MODE_64);
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
    //this->motors->setStallThreshold(0x40);
	this->motors->setAccelerationKVAL(0x96);  //80
	this->motors->setDecelerationKVAL(0x96);  //80
	this->motors->setRunKVAL(0x96);  //B4 70
	this->motors->setHoldKVAL(0x40);  //40
	// Disable BEMF compensation and the FLAG (alarm) pin
	this->motors->setParam(L6470::REG_ADDR_ST_SLP, 0x00);
	this->motors->setParam(L6470::REG_ADDR_FN_SLP_ACC, 0x00);
	this->motors->setParam(L6470::REG_ADDR_FN_SLP_DEC, 0x00);
	this->motors->setParam(L6470::REG_ADDR_ALARM_EN, 0x00);
	RCLCPP_INFO_STREAM(this->get_logger(), "L6470: setup done");
    */
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
    this->disableMotors();
    this->motors->stop();
    //delete this->motors;
	//this->motors->softStop();
	//this->motors->resetDevice();
	//this->resetPin->unset();
}

void MotorsNode::update() {
	std::vector<float> speeds;
	//std::vector<L6470::Direction> dirs;
	// requested speeds in steps per second
    //RCLCPP_INFO_STREAM(this->get_logger(), "Received motors speed: " << this->speedL << ' ' << this->speedR << '\n');
	auto speedLSPS = this->speedL / (2.0 * M_PI) * this->stepsPerRevolution;
	auto speedRSPS = this->speedR / (2.0 * M_PI) * this->stepsPerRevolution;
    //RCLCPP_INFO_STREAM(this->get_logger(), "Calculated motors speed: " << speedLSPS << ' ' << speedRSPS << '\n');
	speeds.emplace_back(speedLSPS);
	speeds.emplace_back(speedRSPS);
	//dirs.emplace_back(this->speedL >= 0.0 ? L6470::DIRECTION_FWD : L6470::DIRECTION_REV);
	//dirs.emplace_back(this->speedR >= 0.0 ? L6470::DIRECTION_FWD : L6470::DIRECTION_REV);
    //RCLCPP_INFO_STREAM(this->get_logger(), "Set motors speed: " << speeds[0] << ' ' << speeds[1] << '\n');
	this->setMotorSpeeds(speeds[0], speeds[1], 0);

    std::vector<long> motorPositions= {this->motors->getPositionLeft(), this->motors->getPositionRight()};
    std::vector<float> motorSpeeds = {this->getMotorSpeedLeft(), this->getMotorSpeedRight()};
	std::vector<motor_status> motorStatuses = {this->getMotorStatusLeft(), getMotorStatusRight()};

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
	statusMessageL.hi_z = motorStatuses[0].status[0];
	statusMessageL.busy = motorStatuses[0].status[1];
	statusMessageL.direction = motorStatuses[0].status[2];
	statusMessageL.motor_stopped = motorStatuses[0].status[3];
	statusMessageL.motor_accelerating = motorStatuses[0].status[4];
	statusMessageL.motor_decelerating = motorStatuses[0].status[5];
	statusMessageL.motor_const_speed = motorStatuses[0].status[6];
	statusMessageL.undervoltage = motorStatuses[0].status[7];
	statusMessageL.thermal_warning = motorStatuses[0].status[8];
	statusMessageL.thermal_shutdown = motorStatuses[0].status[9];
	statusMessageL.overcurrent = motorStatuses[0].status[10];
	statusMessageL.step_loss_a = motorStatuses[0].status[11];
	statusMessageL.step_loss_b = motorStatuses[0].status[12];
	statusMessageR.hi_z = motorStatuses[1].status[0];
	statusMessageR.busy = motorStatuses[1].status[1];
	statusMessageR.direction = motorStatuses[1].status[2];
	statusMessageR.motor_stopped = motorStatuses[1].status[3];
	statusMessageR.motor_accelerating = motorStatuses[1].status[4];
	statusMessageR.motor_decelerating = motorStatuses[1].status[5];
	statusMessageR.motor_const_speed = motorStatuses[1].status[6];
	statusMessageR.undervoltage = motorStatuses[1].status[7];
	statusMessageR.thermal_warning = motorStatuses[1].status[8];
	statusMessageR.thermal_shutdown = motorStatuses[1].status[9];
	statusMessageR.overcurrent = motorStatuses[1].status[10];
	statusMessageR.step_loss_a = motorStatuses[1].status[11];
	statusMessageR.step_loss_b = motorStatuses[1].status[12];

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

void MotorsNode::enableMotors() {
    this->motorsEnabled = true;
}

void MotorsNode::disableMotors() {
    this->motorsEnabled = false;
    this->motorSpeedLeft = 0;
    this->motorSpeedRight = 0;
    this->setMotorSpeeds(0.0, 0.0, true);
}

void MotorsNode::setMotorSpeeds(float speedLeft, float speedRight, bool ignoreAcceleration) {
    // Clip speed values
    auto maxSpeedSPS = this->maxSpeed / 2.0 / M_PI * this->stepsPerRevolution;
    clipValue(speedLeft, maxSpeedSPS);
    clipValue(speedRight, maxSpeedSPS);

    // If needed, invert the speeds
    if (this->invertLeftSpeed) {
        speedLeft = -speedLeft;
    }
    if (this->invertRightSpeed) {
        speedRight = -speedRight;
    }

    // Clip speeds according to acceleration limits
    if (ignoreAcceleration) {
        this->motorSpeedLeft = speedLeft;
        this->motorSpeedRight = speedRight;
    } else {
        if (speedLeft > (this->motorSpeedLeft + this->maxAcceleration)) {
            this->motorSpeedLeft = this->motorSpeedLeft + this->maxAcceleration;
        } else if (speedLeft < (this->motorSpeedLeft - this->maxAcceleration)) {
            this->motorSpeedLeft = this->motorSpeedLeft - this->maxAcceleration;
        } else {
            this->motorSpeedLeft = speedLeft;
        }
        if (speedRight > (this->motorSpeedRight + this->maxAcceleration)) {
            this->motorSpeedRight = this->motorSpeedRight + this->maxAcceleration;
        } else if (speedRight < (this->motorSpeedRight - this->maxAcceleration)) {
            this->motorSpeedRight = this->motorSpeedRight - this->maxAcceleration;
        } else {
            this->motorSpeedRight = speedRight;
        }
    }

    this->motors->setSpeed(this->motorSpeedLeft, this->motorSpeedRight);
}

float MotorsNode::getMotorSpeedLeft() const {
    float inversionMultiplier = this->invertLeftSpeed ? -1 : 1;
    return this->motorSpeedLeft * inversionMultiplier;
}

float MotorsNode::getMotorSpeedRight() const {
    float inversionMultiplier = this->invertRightSpeed ? -1 : 1;
    return this->motorSpeedRight * inversionMultiplier;
}

void MotorsNode::getMotorsStatusRegisters(long &motor0,long &motor1) {
    motor0 = motors->getStatusLeft();
    motor1 = motors->getStatusRight();
}

motor_status MotorsNode::getMotorStatusLeft(){
    long motorStatusLeft = motors->getStatusLeft();
    motor_status status;

    status.status[0] = ~motorStatusLeft & 0x4000;
    status.status[1] = ~motorStatusLeft & 0x2000;
    status.status[2] = ~motorStatusLeft & 0x1000;
    status.status[3] = ~motorStatusLeft & 0x0800;
    status.status[4] = ~motorStatusLeft & 0x0400;
    status.status[5] = ~motorStatusLeft & 0x0200;
    status.status[6] = ~motorStatusLeft & 0x0060;
    status.status[7] = (~motorStatusLeft & 0x0040) & (motorStatusLeft & 0x0020);
    status.status[8] = (motorStatusLeft & 0x0040) & (~motorStatusLeft & 0x0020);
    status.status[9] = motorStatusLeft & 0x0060;
    status.status[10] = motorStatusLeft & 0x0010;
    status.status[11] = ~motorStatusLeft & 0x0002;
    status.status[12] = motorStatusLeft & 0x0001;

    return status;
}

motor_status MotorsNode::getMotorStatusRight(){
    long motorStatusRight = motors->getStatusRight();
    motor_status status;

    status.status[0] = ~motorStatusRight & 0x4000;
    status.status[1] = ~motorStatusRight & 0x2000;
    status.status[2] = ~motorStatusRight & 0x1000;
    status.status[3] = ~motorStatusRight & 0x0800;
    status.status[4] = ~motorStatusRight & 0x0400;
    status.status[5] = ~motorStatusRight & 0x0200;
    status.status[6] = ~motorStatusRight & 0x0060;
    status.status[7] = (~motorStatusRight & 0x0040) & (motorStatusRight & 0x0020);
    status.status[8] = (motorStatusRight & 0x0040) & (~motorStatusRight & 0x0020);
    status.status[9] = motorStatusRight & 0x0060;
    status.status[10] = motorStatusRight & 0x0010;
    status.status[11] = ~motorStatusRight & 0x0002;
    status.status[12] = motorStatusRight & 0x0001;

    return status;
}

/*void MotorsNode::getMotorsSpeedConfiguration(float &max, float &min, float &acc, float &dec){
    max = this->motors->getMaxSpeed();
    min = this->motors->getMinSpeed();
    acc = this->motors->getAcc();
    dec = this->motors->getDec();
}
 */

void clipValue(float & value, float max) {
    if (value > max) {
        value = max;
    } else if (value < -max) {
        value = -max;
    }
}
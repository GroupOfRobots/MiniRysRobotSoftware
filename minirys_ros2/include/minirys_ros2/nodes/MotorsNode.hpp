#pragma once

#include <rclcpp/rclcpp.hpp>

#include <minirys_msgs/msg/motor_command.hpp>
#include <minirys_msgs/msg/motor_driver_status.hpp>
#include <std_msgs/msg/float64.hpp>

//#include <L6470.hpp>
#include <motorsclass.h>
#include <memory>
//#include <SPIBus.hpp>

void clipValue(float & value, float max);

struct motor_status {
    bool status[13];
    motor_status(){
        for (int i; i < 13; i++) status[i] = false;
    };
};

class MotorsNode: public rclcpp::Node {
public:
	RCLCPP_SMART_PTR_DEFINITIONS(MotorsNode);

	explicit MotorsNode(rclcpp::NodeOptions options);

	~MotorsNode() override;

private:
	double stepsPerRevolution;
	double maxSpeed;

	//SPIBus::SharedPtr spi;

	//GPIOPin::SharedPtr resetPin;

	//GPIOPin::SharedPtr busyPin;

    //Motors *motors;
    std::shared_ptr<Motors> motors;

	double speedL;

	double speedR;

    bool motorsEnabled;
    float motorSpeedLeft;
    float motorSpeedRight;
    float maxAcceleration;
    bool invertLeftSpeed;
    bool invertRightSpeed;

	rclcpp::TimerBase::SharedPtr updateTimer;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorPositionLPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorPositionRPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorSpeedLPublisher;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motorSpeedRPublisher;

	rclcpp::Publisher<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusLPublisher;

	rclcpp::Publisher<minirys_msgs::msg::MotorDriverStatus>::SharedPtr motorStatusRPublisher;

	rclcpp::Subscription<minirys_msgs::msg::MotorCommand>::SharedPtr motorCommandSubscription;

	void update();

	void receiveMotorCommand(const minirys_msgs::msg::MotorCommand::SharedPtr message);

    void enableMotors();
    void disableMotors();

    void setMotorSpeeds(float speedLeft, float speedRight, bool ignoreAcceleration = false);
    float getMotorSpeedLeft() const;
    float getMotorSpeedRight() const;

    void getMotorsStatusRegisters(long &motor0, long &motor1);
    motor_status getMotorStatusLeft();
    motor_status getMotorStatusRight();
    //void getMotorsSpeedConfiguration(float &max, float &min, float &acc, float &dec);
};

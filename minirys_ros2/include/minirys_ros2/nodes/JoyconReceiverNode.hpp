#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>


#define JOY_DEV "/dev/input/js0"

class JoyconReceiverNode : public rclcpp::Node{
	public:
        RCLCPP_SMART_PTR_DEFINITIONS(JoyconReceiverNode);
        explicit JoyconReceiverNode(rclcpp::NodeOptions options);
		~JoyconReceiverNode() override;

	private:
		int joy_fd, *axis, *axisPast, num_of_axis, num_of_buttons;
		bool *button, *buttonPast;
		char name_of_joystick[80];
		struct js_event js;

		int forwardAxis, rotationAxis, forwardSpeedFactor, rotationSpeedFactor;
		bool forwardAxisInverted, rotationAxisInverted;

		int standUpButton, layDownButton;
		int servoUpButton, servoDownButton;

		rclcpp::TimerBase::SharedPtr get_joycon_state_timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joycon_control_publisher;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr balance_control_publisher;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_control_publisher;
        geometry_msgs::msg::Twist msg;
		std_msgs::msg::Bool balance_msg;
		std_msgs::msg::Bool servo_msg;

		void get_joycon_state();
};
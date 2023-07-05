#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "rclcpp/rclcpp.hpp"
//#include "../FrequencyCounter/FrequencyCounter.hpp"
//#include "minirys_interfaces/msg/minirys_input.hpp"
#include <geometry_msgs/msg/twist.hpp>

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

		int standUpButton, layDownButton, shutdownButton;//, printStatusButton, printLocationButton;

		//FrequencyCounter *counter;

		rclcpp::TimerBase::SharedPtr get_joycon_state_timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joycon_control_publisher;
        geometry_msgs::msg::Twist msg;
		//rclcpp::Publisher<minirys_interfaces::msg::MinirysInput>::SharedPtr joycon_control_publisher;
		//minirys_interfaces::msg::MinirysInput msg;

		void get_joycon_state();
};
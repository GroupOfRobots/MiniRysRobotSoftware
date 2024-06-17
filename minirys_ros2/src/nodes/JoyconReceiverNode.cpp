#include "minirys_ros2/nodes/JoyconReceiverNode.hpp"

JoyconReceiverNode::JoyconReceiverNode(rclcpp::NodeOptions options): Node("joycon_receiver", options) {
	if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 )
	{
		RCLCPP_ERROR(this->get_logger(), "Couldn't open joystick" );
		return;
	}

	num_of_axis = 0;
	num_of_buttons = 0;
	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	RCLCPP_INFO(this->get_logger(),"Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
	, name_of_joystick
	, num_of_axis
	, num_of_buttons );
	if (num_of_buttons < 50 && num_of_axis < 50) {

		axis = (int *) calloc( num_of_axis, sizeof( int ) );
		axisPast = (int *) calloc( num_of_axis, sizeof( int ) );
		for(int i = 0; i < num_of_axis; i++){
			axis[i] = 0;
			axisPast[i] = 0;
		}
		// RCLCPP_INFO(this->get_logger(), "Debug");
		button = (bool *) calloc( num_of_buttons, sizeof( bool ) );
		buttonPast = (bool *) calloc( num_of_buttons, sizeof( bool ) );
		for(int i = 0; i < num_of_buttons; i++){
			button[i] = false;
			buttonPast[i] = false;
		}
	}

	fcntl( joy_fd, F_SETFL, O_NONBLOCK ); /* use non-blocking mode */

	// Get axis parameters
	this->declare_parameter("forwardAxis", rclcpp::ParameterValue(1));
	forwardAxis = this->get_parameter("forwardAxis").get_value<int>();
	this->declare_parameter("rotationAxis", rclcpp::ParameterValue(2));
	rotationAxis = this->get_parameter("rotationAxis").get_value<int>();
	this->declare_parameter("forwardSpeedFactor", rclcpp::ParameterValue(300000)); //48000
	forwardSpeedFactor = this->get_parameter("forwardSpeedFactor").get_value<int>();
	this->declare_parameter("rotationSpeedFactor", rclcpp::ParameterValue(48000)); //16000
	rotationSpeedFactor = this->get_parameter("rotationSpeedFactor").get_value<int>();
	this->declare_parameter("forwardAxisInverted", rclcpp::ParameterValue(true));
	forwardAxisInverted = this->get_parameter("forwardAxisInverted").get_value<bool>();
	this->declare_parameter("rotationAxisInverted", rclcpp::ParameterValue(false));
	rotationAxisInverted = this->get_parameter("rotationAxisInverted").get_value<bool>();

	// Get button parameters
	this->declare_parameter("standUpButton", rclcpp::ParameterValue(3));
	standUpButton = this->get_parameter("standUpButton").get_value<int>();
	this->declare_parameter("layDownButton", rclcpp::ParameterValue(1));
	layDownButton = this->get_parameter("layDownButton").get_value<int>();
	this->declare_parameter("servoUpButton", rclcpp::ParameterValue(2));
	servoUpButton = this->get_parameter("servoUpButton").get_value<int>();
	this->declare_parameter("servoDownButton", rclcpp::ParameterValue(0));
	servoDownButton = this->get_parameter("servoDownButton").get_value<int>();

    joycon_control_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/minirys3/cmd_vel", 10);
	balance_control_publisher = this->create_publisher<std_msgs::msg::Bool>("/minirys3/balance_mode", 10);
	servo_control_publisher = this->create_publisher<std_msgs::msg::Bool>("/minirys3/servo_status", 10);

    msg = geometry_msgs::msg::Twist();

	this->declare_parameter("period", rclcpp::ParameterValue(200));
	get_joycon_state_timer = this->create_wall_timer(
	std::chrono::milliseconds(this->get_parameter("period").get_value<int>()), std::bind(&JoyconReceiverNode::get_joycon_state, this));
	RCLCPP_INFO(this->get_logger(), "Joycon receiver initialized.");
}

JoyconReceiverNode::~JoyconReceiverNode(){
	close( joy_fd );
}

void JoyconReceiverNode::get_joycon_state() {
	/* read the joystick state */
	/* see what to do with the event */
	while(read(joy_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
		switch (js.type & ~JS_EVENT_INIT)
		{
			case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				// if (axis [ js.number ] != axisPast[js.number]) RCLCPP_INFO(this->get_logger(),"Axis %d:\t%d", js.number, axis[js.number]);
				axisPast[js.number] = axis[js.number];
				break;
			case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				if (button [ js.number ] != buttonPast[js.number]) RCLCPP_INFO(this->get_logger(),"Button %d:\t%d", js.number, button[js.number]);
				buttonPast[js.number] = button[js.number];
				break;
		}
	}

    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;

	if(!forwardAxisInverted){
        msg.linear.x  = axis[forwardAxis]/(forwardSpeedFactor*1.0);
	} else {
        msg.linear.x = -axis[forwardAxis]/(forwardSpeedFactor*1.0);
	}

	if(!rotationAxisInverted){
        msg.angular.z = -axis[rotationAxis]/(rotationSpeedFactor*1.0);
	} else {
        msg.angular.z = axis[rotationAxis]/(rotationSpeedFactor*1.0);
	}

	balance_msg = std_msgs::msg::Bool();
	servo_msg = std_msgs::msg::Bool();
	if (button[standUpButton] == 1)
	{
		balance_msg.data = true;
		balance_control_publisher->publish(balance_msg);
	}
	if (button[layDownButton] == 1)
	{
		balance_msg.data = false;
		balance_control_publisher->publish(balance_msg);
	}
	if (button[servoUpButton] == 1)
	{
		servo_msg.data = true;
		servo_control_publisher->publish(servo_msg);
	}
	if (button[servoDownButton] == 1)
	{
		servo_msg.data = false;
		servo_control_publisher->publish(servo_msg);
	}

	joycon_control_publisher->publish(msg);
}
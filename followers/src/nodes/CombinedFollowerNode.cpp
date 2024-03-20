#include "nodes/CombinedFollowerNode.hpp"


using namespace std::chrono_literals;


CombinedFollower::CombinedFollower() : Node("combined_follower") {
    this->declare_parameter("minirys_namespace", rclcpp::ParameterValue("minirys"));

    this->declare_parameter("corridor_recogniton_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("end_side_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("end_front_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("corridor_out_front_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("corridor_out_side_dist", rclcpp::ParameterValue(0.0));
    this->declare_parameter("stabilization_time", rclcpp::ParameterValue(0.0));
    this->declare_parameter("standing_up_time", rclcpp::ParameterValue(0.0));
    this->declare_parameter("turning_time", rclcpp::ParameterValue(0.0));
    this->declare_parameter("positioning_time", rclcpp::ParameterValue(0.0));
    std::this_thread::sleep_for(100ms);

    corridor_recogniton_dist = this->get_parameter("corridor_recogniton_dist").as_double();
    end_side_dist = this->get_parameter("end_side_dist").as_double();
    end_front_dist = this->get_parameter("end_front_dist").as_double();
    corridor_out_front_dist = this->get_parameter("corridor_out_front_dist").as_double();
    corridor_out_side_dist = this->get_parameter("corridor_out_side_dist").as_double();
    stabilization_time = this->get_parameter("stabilization_time").as_int();
    standing_up_time = this->get_parameter("standing_up_time").as_int();
    turning_time = this->get_parameter("turning_time").as_int();
    positioning_time = this->get_parameter("positioning_time").as_int();

    std::string minirys_namespace = this->get_parameter("minirys_namespace").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: minirys_namespace " << minirys_namespace);

    subscription_dist5_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_5", 10, std::bind(&CombinedFollower::left_sensor_callback, this, std::placeholders::_1));

    subscription_dist2_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_2", 10, std::bind(&CombinedFollower::right_sensor_callback, this, std::placeholders::_1));

    subscription_dist3_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_3", 10, std::bind(&CombinedFollower::front_sensor_callback, this, std::placeholders::_1));
    
    publisher_bool_wallf_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/is_wall_follower", 10);
    publisher_bool_linef_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/is_line_follower", 10);
    publisher_bool_balance_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/balance_mode", 10);
    publisher_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("/"+minirys_namespace+"/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&CombinedFollower::timer_callback, this));
    }

CombinedFollower::~CombinedFollower(){
	auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
	publisher_velocity_->publish(*msg_twist);
}

int CombinedFollower::getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_measure_time).count();
}


void CombinedFollower::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->left_sensor = (float) msg->range;

}

void CombinedFollower::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->right_sensor = (float) msg->range;
}

void CombinedFollower::front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->front_sensor = (float) msg->range;
}

void CombinedFollower::timer_callback() {
    auto msg_bool = std::make_shared<std_msgs::msg::Bool>();
    auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
    if(flag_ == INITIAL_STATE){
        start_measure_ = std::chrono::high_resolution_clock::now();
        flag_ = START_FOLLOWING_LINE;
    }

    if(flag_ == START_FOLLOWING_LINE && getTimeToNow(start_measure_) > INIT_TIME){
        msg_bool->data = true;
        publisher_bool_linef_->publish(*msg_bool);
        msg_bool->data = false;
        publisher_bool_wallf_->publish(*msg_bool);
        flag_ = CORRIDOR_BEGIN;
    }
    
    if(right_sensor < corridor_recogniton_dist && left_sensor < corridor_recogniton_dist && flag_ == CORRIDOR_BEGIN){
        flag_ = GO_STRAIGHT;

        msg_bool->data = false;
        publisher_bool_linef_->publish(*msg_bool);
        
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    if(flag_== GO_STRAIGHT && getTimeToNow(start_measure_) > positioning_time){
        flag_ = STANDING_UP;

        msg_bool->data = true;
        publisher_velocity_->publish(*msg_twist);
        publisher_bool_balance_->publish(*msg_bool);
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    if(getTimeToNow(start_measure_) > standing_up_time && flag_== STANDING_UP){
        msg_bool->data = true;
        publisher_bool_wallf_->publish(*msg_bool);
        flag_ = START_FOLLOWING_WALL;
    }

    if(right_sensor < end_side_dist && left_sensor < end_side_dist && front_sensor < end_front_dist && flag_== START_FOLLOWING_WALL){
        msg_bool->data = false;
        publisher_bool_wallf_->publish(*msg_bool);
        publisher_velocity_->publish(*msg_twist);
        flag_ = STOP;
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    
    if(flag_ == STOP && getTimeToNow(start_measure_) > stabilization_time){
        start_measure_ = std::chrono::high_resolution_clock::now();
        flag_ = TURN_BACK;
        msg_twist->angular.z = TURN_VAL;
        publisher_velocity_->publish(*msg_twist);
    }
    if(flag_ == TURN_BACK && getTimeToNow(start_measure_) > turning_time){
        flag_ = STABILISE;
        start_measure_ = std::chrono::high_resolution_clock::now();
        publisher_velocity_->publish(*msg_twist);
        
    }

    if(flag_ == STABILISE && getTimeToNow(start_measure_) > stabilization_time){
        msg_bool->data = true;
        flag_ = CONTINUE_FOLLOWING_WALL;
        publisher_bool_wallf_->publish(*msg_bool);
    }

    if(flag_ == CONTINUE_FOLLOWING_WALL && ((left_sensor > corridor_out_side_dist && right_sensor > end_front_dist) ||
    (left_sensor > end_front_dist && right_sensor > corridor_out_side_dist)) && front_sensor > corridor_out_front_dist){
        msg_bool->data = false;
        publisher_bool_wallf_->publish(*msg_bool);
        flag_ = END_OF_CORRIDOR;
        publisher_velocity_->publish(*msg_twist);
    }

}




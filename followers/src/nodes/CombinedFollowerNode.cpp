#include "nodes/CombinedFollowerNode.hpp"


using namespace std::chrono_literals;


CombinedFollower::CombinedFollower() : Node("combined_follower") {
    this->declare_parameter("minirys_namespace", rclcpp::ParameterValue("minirys"));
    std::this_thread::sleep_for(100ms);

    std::string minirys_namespace = this->get_parameter("minirys_namespace").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: minirys_namespace " << minirys_namespace);

    subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_5", 10, std::bind(&CombinedFollower::left_sensor_callback, this, std::placeholders::_1));

    subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_2", 10, std::bind(&CombinedFollower::right_sensor_callback, this, std::placeholders::_1));

    subscription3_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_3", 10, std::bind(&CombinedFollower::front_sensor_callback, this, std::placeholders::_1));
    
    publisher1_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/is_wall_follower", 10);
    publisher2_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/is_line_follower", 10);
    publisher3_ = this->create_publisher<std_msgs::msg::Bool>("/"+minirys_namespace+"/balance_mode", 10);
    publisher4_ = this->create_publisher<geometry_msgs::msg::Twist>("/"+minirys_namespace+"/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&CombinedFollower::timer_callback, this));
    }

CombinedFollower::~CombinedFollower(){
	auto msg2 = std::make_shared<geometry_msgs::msg::Twist>();
	publisher4_->publish(*msg2);
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
    auto msg = std::make_shared<std_msgs::msg::Bool>();
    auto msg2 = std::make_shared<geometry_msgs::msg::Twist>();
    if(flag_ == INITIAL_STATE){
        start_measure_ = std::chrono::high_resolution_clock::now();
        flag_ = START_FOLLOWING_LINE;
    }

    if(flag_ == START_FOLLOWING_LINE && getTimeToNow(start_measure_) > 500){
        msg->data = true;
        publisher2_->publish(*msg);
        msg->data = false;
        publisher1_->publish(*msg);
        flag_ = CORRIDOR_BEGIN;
    }
    
    if(right_sensor < 0.200 && left_sensor < 0.2 && flag_ == CORRIDOR_BEGIN){
        flag_ = GO_STRAIGHT;

        msg->data = false;
        publisher2_->publish(*msg);
        
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    if(flag_== GO_STRAIGHT && getTimeToNow(start_measure_) > 1500){
        flag_ = STANDING_UP;

        msg->data = true;
        publisher4_->publish(*msg2);
        publisher3_->publish(*msg);
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    if(getTimeToNow(start_measure_) > 7000 && flag_== STANDING_UP){
        msg->data = true;
        publisher1_->publish(*msg);
        flag_ = START_FOLLOWING_WALL;
    }

    if(right_sensor < 0.3 && left_sensor < 0.3 && front_sensor < 0.21 && flag_== START_FOLLOWING_WALL){
        msg->data = false;
        publisher1_->publish(*msg);
        publisher4_->publish(*msg2);
        flag_ = STOP;
        start_measure_ = std::chrono::high_resolution_clock::now();
    }
    
    if(flag_ == STOP && getTimeToNow(start_measure_) > 4000){
        start_measure_ = std::chrono::high_resolution_clock::now();
        flag_ = TURN_BACK;
        msg2->angular.z = 1.57;
        publisher4_->publish(*msg2);
    }
    if(flag_ == TURN_BACK && getTimeToNow(start_measure_) > 2000){
        flag_ = STABILISE;
        start_measure_ = std::chrono::high_resolution_clock::now();
        publisher4_->publish(*msg2);
        
    }

    if(flag_ == STABILISE && getTimeToNow(start_measure_) > 4000){
        msg->data = true;
        flag_ = CONTINUE_FOLLOWING_WALL;
        publisher1_->publish(*msg);
    }

    if(flag_ == CONTINUE_FOLLOWING_WALL && ((left_sensor > 0.4 && right_sensor > 0.23) ||
    (left_sensor > 0.23 && right_sensor > 0.4)) && front_sensor > 0.7){
        msg->data = false;
        publisher1_->publish(*msg);
        flag_ = END_OF_CORRIDOR;
        publisher4_->publish(*msg2);
    }

}




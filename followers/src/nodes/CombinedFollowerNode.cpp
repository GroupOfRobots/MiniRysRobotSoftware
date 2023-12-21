#include "nodes/CombinedFollowerNode.hpp"


using namespace std::chrono_literals;


CombinedFollower::CombinedFollower() : Node("combined_follower") {

        subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_5", 10, std::bind(&CombinedFollower::left_sensor_callback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_2", 10, std::bind(&CombinedFollower::right_sensor_callback, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_3", 10, std::bind(&CombinedFollower::front_sensor_callback, this, std::placeholders::_1));
        
        publisher1_ = this->create_publisher<std_msgs::msg::Bool>("/is_wall_follower", 10);
        publisher2_ = this->create_publisher<std_msgs::msg::Bool>("/is_line_follower", 10);
        publisher3_ = this->create_publisher<std_msgs::msg::Bool>("/minirys/balance_mode", 10);
        publisher4_ = this->create_publisher<geometry_msgs::msg::Twist>("/minirys/cmd_vel", 10);

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
        if(flag_ == -1){
            start_measure_ = std::chrono::high_resolution_clock::now();
            flag_ = 0;
        }

        if(flag_ == 0 && getTimeToNow(start_measure_) > 500){
            msg->data = true;
            publisher2_->publish(*msg);
            msg->data = false;
            publisher1_->publish(*msg);
            flag_ = 1;
        }
        
        if(right_sensor < 0.200 && left_sensor < 0.2 && flag_ == 1){
            flag_ = 2;

            msg->data = false;
            publisher2_->publish(*msg);
            
            start_measure_ = std::chrono::high_resolution_clock::now();
        }
        if(flag_==2 && getTimeToNow(start_measure_) > 1500){
            flag_ = 3;

            msg->data = true;
            publisher4_->publish(*msg2);
            publisher3_->publish(*msg);
            start_measure_ = std::chrono::high_resolution_clock::now();
        }
        if(getTimeToNow(start_measure_) > 7000 && flag_==3){
            msg->data = true;
            publisher1_->publish(*msg);
            flag_ = 4;
        }

        if(right_sensor < 0.3 && left_sensor < 0.3 && front_sensor < 0.20 && flag_==4){
            msg->data = false;
            publisher1_->publish(*msg);
            publisher4_->publish(*msg2);
            flag_ = 5;
            start_measure_ = std::chrono::high_resolution_clock::now();
        }
        
        if(flag_ == 5 && getTimeToNow(start_measure_) > 4000){
            start_measure_ = std::chrono::high_resolution_clock::now();
            flag_ = 6;
            msg2->angular.z = 1.57;
            publisher4_->publish(*msg2);
        }
        if(flag_ == 6 && getTimeToNow(start_measure_) > 1800){
            flag_ = 7;
            start_measure_ = std::chrono::high_resolution_clock::now();
            publisher4_->publish(*msg2);
            
        }

        if(flag_ == 7 && getTimeToNow(start_measure_) > 4000){
            msg->data = true;
            flag_ = 8;
            publisher1_->publish(*msg);
        }

        if(flag_ == 8 && ((left_sensor > 0.4 && right_sensor > 0.23) ||
        (left_sensor > 0.23 && right_sensor > 0.4)) && front_sensor > 0.7){
            msg->data = false;
            publisher1_->publish(*msg);
            flag_ = 9;
            publisher4_->publish(*msg2);
        }

    }




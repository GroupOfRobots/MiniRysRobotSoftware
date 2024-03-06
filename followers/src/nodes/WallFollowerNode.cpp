#include "nodes/WallFollowerNode.hpp"


using namespace std::chrono_literals;


WallFollower::WallFollower() : Node("wall_follower") {
    //declare parameters
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    this->declare_parameter("K", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Ti", rclcpp::ParameterValue(0.0));
    this->declare_parameter("Td", rclcpp::ParameterValue(0.0));
    this->declare_parameter("linearSpeed", rclcpp::ParameterValue(0.0));
    this->declare_parameter("maxU", rclcpp::ParameterValue(0.9));
    this->declare_parameter("minirys_namespace", rclcpp::ParameterValue("minirys"));
    std::this_thread::sleep_for(100ms);
    
    //load parameters
    double timer_period = this->get_parameter("timer_period").as_double();
    double K = this->get_parameter("K").as_double();
    double Ti = this->get_parameter("Ti").as_double();
    double Td = this->get_parameter("Td").as_double();
    std::string minirys_namespace = this->get_parameter("minirys_namespace").as_string();
    this->linearSpeed = this->get_parameter("linearSpeed").as_double();
    this->pid = std::unique_ptr<PIDRegulator>(new PIDRegulator((float) timer_period,(float) K,(float) Ti,(float) Td));
    this->maxU = (float)this->get_parameter("maxU").as_double();

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Ti " << Ti);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Td " << Td);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: K " << K);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << this->linearSpeed);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: maxU " << this->maxU);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: minirys_namespace " << minirys_namespace);
    //publishers
    publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/"+minirys_namespace+"/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&WallFollower::timer_callback, this));
    //subscribers
    subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_5", 10, std::bind(&WallFollower::left_sensor_callback, this, std::placeholders::_1));
    subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_2", 10, std::bind(&WallFollower::right_sensor_callback, this, std::placeholders::_1));
    subscription3_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/"+minirys_namespace+"/internal/distance_3", 10, std::bind(&WallFollower::front_sensor_callback, this, std::placeholders::_1));
    subscription4_ = this->create_subscription<std_msgs::msg::Bool>(
    "/"+minirys_namespace+"/is_wall_follower", 10, std::bind(&WallFollower::is_callback, this, std::placeholders::_1));

    program_start_ = std::chrono::high_resolution_clock::now();
    
    }

    
int WallFollower::getTimeToNow(std::chrono::time_point<std::chrono::high_resolution_clock> start_measure_time){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_measure_time).count();
}

void WallFollower::timer_callback() {
    if(this->is_working_ && getTimeToNow(program_start_) > 2000){
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        float u = 0.0f;
        msg->linear.y = this->linearSpeed;
        if (this->flag_ == 1){
            u = this->pid->pid_aw(this->right_sensor-this->left_sensor,0,20.0f, this->maxU);
            if (u >this->maxU){
                u = this->maxU;
            }
            else if(u < -this->maxU){
                u = -this->maxU;
            }
        }
        else if(this->flag_ == 2){
            u = -1.2;
            msg->linear.y = -1;
        }
        else if(this->flag_ == 3){
            u = 1.2;
            msg->linear.y = -1;
        }
        else if(this->flag_ == 4){
            u = -0.1;
            
        }
        else if(this->flag_ == 5){
            u = 0.1;
            
        }

        if(((this->right_sensor > 0.320 && this->left_sensor < 0.260 && this->front_sensor < 0.300) || (this->right_sensor > this->left_sensor && this->right_sensor-this->left_sensor > 0.12 && this->left_sensor> 0.26))){
            this->flag_ = 2;
        }

        if(((this->right_sensor < 0.260 && this->left_sensor > 0.320 && this->front_sensor < 0.300) || (this->right_sensor < this->left_sensor && this->right_sensor-this->left_sensor < -0.12 && this->right_sensor > 0.26))){
            this->flag_ = 3;
        }

        if(this->front_sensor > 0.400 && this->flag_ == 3){
            this->flag_ = 5;
        }

        if(this->front_sensor > 0.400 && this->flag_ == 2){
            this->flag_ = 4;
        }

        if((this->flag_ == 4 || this->flag_ == 5) &&  this->right_sensor <= 0.260 && this->left_sensor <= 0.260){
            this->flag_ = 1;
            this->pid->clear();
        }

        msg->angular.z = u;
        publisher_->publish(*msg);
    }
}

WallFollower::~WallFollower(){
	auto msg = std::make_shared<geometry_msgs::msg::Twist>();
	publisher_->publish(*msg);

}

void WallFollower::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->left_sensor = (float) msg->range + 0.02;

}

void WallFollower::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->right_sensor = (float) msg->range;
}

void WallFollower::front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
{
    this->front_sensor = (float) msg->range;
}

void WallFollower::is_callback(const std_msgs::msg::Bool::SharedPtr msg){
    this->is_working_ = msg->data;
}


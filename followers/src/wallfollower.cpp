#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include "pid.cpp"
#include <memory>



using namespace std::chrono_literals;

class WallFollower: public rclcpp::Node{
    public:
    WallFollower() : Node("wall_follower") {

        this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
        this->declare_parameter("K", rclcpp::ParameterValue(0.0));
        this->declare_parameter("Ti", rclcpp::ParameterValue(0.0));
        this->declare_parameter("Td", rclcpp::ParameterValue(0.0));
        this->declare_parameter("linearSpeed", rclcpp::ParameterValue(0.0));
        this->declare_parameter("maxU", rclcpp::ParameterValue(0.9));

        // Get and save/use the parameters
        std::this_thread::sleep_for(100ms);
        

        double timer_period = this->get_parameter("timer_period").as_double();
        double K = this->get_parameter("K").as_double();
        double Ti = this->get_parameter("Ti").as_double();
        double Td = this->get_parameter("Td").as_double();
        this->linearSpeed = this->get_parameter("linearSpeed").as_double();
        this->pid = std::unique_ptr<PID>(new PID((float) timer_period,(float) K,(float) Ti,(float) Td));
        this->maxU = (float)this->get_parameter("maxU").as_double();

        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Ti " << Ti);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: Td " << Td);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: K " << K);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: linear speed " << this->linearSpeed);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);
        RCLCPP_INFO_STREAM(this->get_logger(), "Got param: maxU " << this->maxU);

        publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/minirys/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&WallFollower::timer_callback, this));

        subscription1_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_5", 10, std::bind(&WallFollower::left_sensor_callback, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_2", 10, std::bind(&WallFollower::right_sensor_callback, this, std::placeholders::_1));

        subscription3_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/minirys/internal/distance_3", 10, std::bind(&WallFollower::front_sensor_callback, this, std::placeholders::_1));

    }

    private:

    void timer_callback() {
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        float u;
        if (this->flag_ == 1){
            u = this->pid->pid_aw(this->right_sensor-this->left_sensor,0,20.0f, this->maxU);
            if (u >this->maxU){
                u = this->maxU;
            }
            else if(u < -this->maxU){
                u = -this->maxU;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "y:  " << this->right_sensor-this->left_sensor);
            msg->linear.y = this->linearSpeed;
        }
        else if(this->flag_ == 2){
            RCLCPP_INFO_STREAM(this->get_logger(), "Right turn  " );
            u = -1.2;
            msg->linear.y = 0;
        }
        else if(this->flag_ == 3){
            RCLCPP_INFO_STREAM(this->get_logger(), "Left turn  " );
            u = 1.2;
            msg->linear.y = 0;
        }
        else if(this->flag_ == 4){
            RCLCPP_INFO_STREAM(this->get_logger(), "Go str  " );
            u = -0.1;
            msg->linear.y = this->linearSpeed;
        }
        else if(this->flag_ == 5){
            RCLCPP_INFO_STREAM(this->get_logger(), "Go str  " );
            u = 0.1;
            msg->linear.y = this->linearSpeed;
        }


        if(this->right_sensor > 0.300 && this->left_sensor < 0.210 && this->front_sensor < 0.250){
            this->flag_ = 2;
        }

        if(this->right_sensor < 0.210 && this->left_sensor > 0.300 && this->front_sensor < 0.250){
            this->flag_ = 3;
        }

        if(this->front_sensor > 0.300 && this->flag_ == 3){
            this->flag_ = 5;
        }

        if(this->front_sensor > 0.300 && this->flag_ == 2){
            this->flag_ = 4;
        }
        if((this->flag_ == 4 || this->flag_ == 5) &&  this->right_sensor <= 0.210 && this->left_sensor <= 0.210){
            this->flag_ = 1;
            this->pid->clear();
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "u:  " << u);
        RCLCPP_INFO_STREAM(this->get_logger(), "front:  " << this->front_sensor);
        msg->linear.y = this->linearSpeed;
        msg->angular.z = u;
        publisher_->publish(*msg);
        
    }

    void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->left_sensor = (float) msg->range;

    }

    void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->right_sensor = (float) msg->range;
    }

    void front_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        this->front_sensor = (float) msg->range;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription3_;

    float left_sensor = 0.0f;
    float right_sensor = 0.0f;
    float front_sensor = 0.0f;

    std::unique_ptr<PID> pid;
    double linearSpeed;
    float maxU;
    int flag_ = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}

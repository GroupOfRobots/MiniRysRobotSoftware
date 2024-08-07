#include "nodes/DetectorNode.hpp"


using namespace std::chrono_literals;


Detector::Detector() : Node("detector") 
{
    //declare parameters
    this->declare_parameter("target_size", rclcpp::ParameterValue(650));
    this->declare_parameter("weight", rclcpp::ParameterValue("weights/best100ep1inp.ncnn"));
    this->declare_parameter("prob_threshold", rclcpp::ParameterValue(0.25));
    this->declare_parameter("nms_threshold", rclcpp::ParameterValue(0.45));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    std::this_thread::sleep_for(100ms);
    
    //load parameters
    double timer_period = this->get_parameter("timer_period").as_double();
    this->prob_threshold = (float) this->get_parameter("prob_threshold").as_double();
    this->nms_threshold = (float) this->get_parameter("nms_threshold").as_double();
    int target_size = this->get_parameter("target_size").as_int();
    std::string weight = this->get_parameter("weight").as_string();
    this->yolov7_.load(target_size, weight+".param", weight + ".bin");

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: prob_threshold " << this->prob_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: nms_threshold " << this->nms_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: target_size " << target_size);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: weight " << weight);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);

    //publishers
    publisher_detected_ =
    this->create_publisher<sensor_msgs::msg::Image>("/img_detect", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&Detector::timer_callback, this));
    //subscribers
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/TODO", 10, std::bind(&Detector::image_callback, this, std::placeholders::_1));
}

void Detector::timer_callback() 
{
    detected_img_ = ori_img_;
    std::vector<Object> objects;
    yolov7_->detect(detected_img_, objects, prob_threshold, nms_threshold);
    yolov7_->draw(detected_img_, objects);
    auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", detected_img_).toImageMsg();
    publisher_detected_->publish(*msg_img);
}

void Detector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    ori_img_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
}

Detector::~Detector(){
}

#include "shuttlecock_detect/nodes/DetectorNode.hpp"


using namespace std::chrono_literals;


Detector::Detector() : Node("detector") 
{
    //declare parameters
    this->declare_parameter("target_size", rclcpp::ParameterValue(650));
    this->declare_parameter("weights", rclcpp::ParameterValue("best100ep1inp.ncnn"));
    this->declare_parameter("prob_threshold", rclcpp::ParameterValue(0.25));
    this->declare_parameter("nms_threshold", rclcpp::ParameterValue(0.45));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    this->declare_parameter("width_front", rclcpp::ParameterValue(0.05));
    this->declare_parameter("focal_length", rclcpp::ParameterValue(0.05));
    this->declare_parameter("width_side", rclcpp::ParameterValue(0.05));
    std::this_thread::sleep_for(100ms);
    
    //load parameters
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("shuttlecock_detect");
    double timer_period = this->get_parameter("timer_period").as_double();
    prob_threshold_ = (float) this->get_parameter("prob_threshold").as_double();
    nms_threshold_ = (float) this->get_parameter("nms_threshold").as_double();
    int target_size = this->get_parameter("target_size").as_int();
    width_front_ = (float) this->get_parameter("width_front").as_double();
    focal_length_ = (float) this->get_parameter("focal_length").as_double();
    width_side_ = (float) this->get_parameter("width_side").as_double();
    std::string weight = this->get_parameter("weights").as_string();
    yolov7_ = std::make_unique<YoloV7>(); 
    yolov7_->load(target_size, package_share_directory+ "/weights/" + weight+".param",
        package_share_directory+ "/weights/" +weight + ".bin");

    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: prob_threshold " << prob_threshold_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: nms_threshold " << nms_threshold_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: target_size " << target_size);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: weight " << weight);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: width_front " << width_front_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: width_side " << width_side_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: focal_length " << focal_length_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Got param: timer_period " << timer_period);

    //publishers
    publisher_detected_ =
    this->create_publisher<sensor_msgs::msg::Image>("/img_detect", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&Detector::timer_callback, this));
    //subscribers
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/cv_video_frames_plain_img", 10, std::bind(&Detector::image_callback, this, std::placeholders::_1));
}

void Detector::timer_callback() 
{   
    if( !ori_img_.empty() )
    {
        detected_img_ = ori_img_;
        std::vector<Object> objects;
        yolov7_->detect(detected_img_, objects, prob_threshold_, nms_threshold_);
        yolov7_->draw(detected_img_, objects);
        auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", detected_img_).toImageMsg();
        publisher_detected_->publish(*msg_img);
        float dist;
        float deltX;
        float focal_px = focal_length_ * (float)detected_img_.size().width/3.6;
        if(objects.size() >0)
        {
            if( objects[0].label == 0 )
            {
                dist = (width_side_ * focal_px)/ objects[0].rect.width;
            }
            else
            {
                dist = (width_front_ * focal_px)/ objects[0].rect.width;
            }
            deltX = dist/focal_length_ *(3.6/detected_img_.size().width) * ( (float)objects[0].rect.x + (float)objects[0].rect.width/2.0 - (float)detected_img_.size().width/2.0);
            std::cout<<"dist: "<< dist<<" x: "<<deltX<<std::endl;
        }
    }
}

void Detector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    ori_img_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
}


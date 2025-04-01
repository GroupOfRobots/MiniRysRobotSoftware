#include "shuttlecock_detect/nodes/DistancesNode.hpp"


using namespace std::chrono_literals;


Distances::Distances(rclcpp::NodeOptions options) : Node("distances", options) 
{
    //declare parameters
    this->declare_parameter("target_size", rclcpp::ParameterValue(650));
    this->declare_parameter("weights", rclcpp::ParameterValue("best100ep1inp.ncnn"));
    this->declare_parameter("prob_threshold", rclcpp::ParameterValue(0.25));
    this->declare_parameter("nms_threshold", rclcpp::ParameterValue(0.45));
    this->declare_parameter("timer_period", rclcpp::ParameterValue(0.05));
    this->declare_parameter("width_front", rclcpp::ParameterValue(68.0));
    this->declare_parameter("focal_length", rclcpp::ParameterValue(3.15));
    this->declare_parameter("width_side", rclcpp::ParameterValue(95.0));
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
    publisher_dat_ = this->create_publisher<btcpp_ros2_interfaces::msg::DistancesAndTransform>("distances", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&Distances::timer_callback, this));

    //subscribers
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/cv_video_frames_plain_img", 10, std::bind(&Distances::image_callback, this, std::placeholders::_1));

    //actions
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Distances::timer_callback() 
{   
    auto msg = btcpp_ros2_interfaces::msg::DistancesAndTransform();
    if( !ori_img_.empty() )
    {
        try
        {
            msg.transform = tf_buffer_->lookupTransform("minirys2/map", "minirys2/base_link", tf2::TimePointZero);
            std::pair<float, float> distances = calculate_dist();
            msg.distance = distances.first;
            msg.delt_x = distances.second;
            publisher_dat_->publish(msg);
        }
        catch (...)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Problems with TF" );
        }
    }
    else
    {
        msg.distance = -1.0f;
        publisher_dat_->publish(msg);
    }
}

void Distances::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    cv::rotate(cv_bridge::toCvCopy(msg, "rgb8")->image, ori_img_, cv::ROTATE_180);
}

std::pair<float, float> Distances::calculate_dist()
{
    detected_img_ = ori_img_;
    std::vector<Object> objects;
    yolov7_->detect(detected_img_, objects, prob_threshold_, nms_threshold_);
    yolov7_->draw(detected_img_, objects);
    float dist, deltX;
    float focal_px = focal_length_ * (float)detected_img_.size().width/3.6;
    if(objects.size() > 0)
    {
        if( objects[0].label == 0 )
        {
            dist = (width_side_ * focal_px)/ objects[0].rect.width;
        }
        else
        {
            dist = (width_front_ * focal_px)/ objects[0].rect.width;
        }
        deltX = dist/focal_length_ *(3.6/detected_img_.size().width) * 
            ( (float)objects[0].rect.x + (float)objects[0].rect.width/2.0 - (float)detected_img_.size().width/2.0);
        return std::make_pair(dist/1000.0, deltX/1000.0 );
    }
    return std::make_pair(-1.0f, -1.0f);
}

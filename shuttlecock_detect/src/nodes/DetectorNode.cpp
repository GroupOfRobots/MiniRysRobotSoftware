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
    publisher_detected_ = this->create_publisher<sensor_msgs::msg::Image>("/img_detect", 10);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period), std::bind(&Detector::timer_callback, this));
    publisher_goal_= this->create_publisher<geometry_msgs::msg::PoseStamped>("minirys2/goal_pose", 10);
    publisher_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("minirys2cmd_vel", 10);
    publisher_isCoverage_ =  this->create_publisher<std_msgs::msg::Bool>("coverage", 10);

    //subscribers
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/cv_video_frames_plain_img", 10, std::bind(&Detector::image_callback, this, std::placeholders::_1));

    //actions
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "minirys2/navigate_to_pose");

    state_ = DETECTING;
    counter_ = 0;
    closer_counter_ = 0;
    is_closer_ = false;
    ori_dist_ = 0;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // TODO rozpocznij krążenie
}

void Detector::timer_callback() 
{   
    if( !ori_img_.empty() )
    {
        if(state_ == DETECTING)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "DETECTING" );
            std::pair<float, float> distances = calculate_dist();
            if(distances.first != -1.0)
            {
                action_client_->async_cancel_all_goals();
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform("minirys2/map", "minirys2/base_link", tf2::TimePointZero);
                RCLCPP_INFO_STREAM(this->get_logger(), "dist: "<< distances.first<<" x: "<<distances.second );
                RCLCPP_INFO_STREAM(this->get_logger(), "currentOdomX: "<< transform_stamped.transform.translation.x<<" currentOdomY: "<<transform_stamped.transform.translation.y );
                state_ = GETTING_CLOSER;
                ori_dist_ = distances.first;
                // TODO Push Goal
                tf2::Quaternion q(
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z,
                    transform_stamped.transform.rotation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                double x_offset = distances.first * std::cos(yaw) + distances.second * std::sin(yaw);
                double y_offset = distances.first * std::sin(yaw) - distances.second * std::cos(yaw);
                auto msg = geometry_msgs::msg::PoseStamped();
                msg.header.frame_id = "minirys2/map";
                msg.header.stamp = this->get_clock()->now();
                msg.pose.position.x = transform_stamped.transform.translation.x + x_offset;
                msg.pose.position.y = transform_stamped.transform.translation.y + y_offset;
                msg.pose.position.z = transform_stamped.transform.translation.z;
                msg.pose.orientation = transform_stamped.transform.rotation;
                publisher_goal_->publish(msg);
                is_goal_reached_ = false;
            }
        }
        else if(state_ == GETTING_CLOSER)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "GETTING_CLOSER" );
            ++counter_;
            std::pair<float, float> distances = calculate_dist();
            if(distances.first != -1.0)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "dist: "<< distances.first<<" x: "<<distances.second );
                closer_counter_ = ori_dist_ >= distances.first? closer_counter_ +1:closer_counter_ ;
                if(closer_counter_ > 1 && counter_ > 1) 
                {
                    is_closer_ = true;
                }
            }
            if(counter_ == 4)
            {
                if(is_closer_)
                {
                    state_ = WAITING_FOR_ARRIVAL;
                }
                else
                {
                    state_ = DETECTING;
                    // TODO przestań jechać do celu i zacznij szukać
                    action_client_->async_cancel_all_goals();
                    RCLCPP_INFO_STREAM(this->get_logger(), "Sent cancel request for navigation goal.");
                    auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>();
                    publisher_velocity_->publish(*msg_twist);
                }
                counter_ = 0;
                is_closer_ = false;
            }
        }
        else if(state_ == WAITING_FOR_ARRIVAL)
        {
            //RCLCPP_INFO_STREAM(this->get_logger(), "WAITING_FOR_ARRIVAL" );
            //TODO oczekiwanie na akcje
        }
    }
}

void Detector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    cv::rotate(cv_bridge::toCvCopy(msg, "rgb8")->image, ori_img_, cv::ROTATE_180);
}

std::pair<float, float> Detector::calculate_dist()
{
    detected_img_ = ori_img_;
    std::vector<Object> objects;
    yolov7_->detect(detected_img_, objects, prob_threshold_, nms_threshold_);
    yolov7_->draw(detected_img_, objects);
    auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", detected_img_).toImageMsg();
    publisher_detected_->publish(*msg_img);
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

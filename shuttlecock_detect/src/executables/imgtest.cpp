#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
    : Node("image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cv_video_frames", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Publish at 10Hz
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        frame = cv::imread("/home/jakub/Documents/ncnnYolov7/image_29_1717764171-923069_jpg.rf.d3d04ad129cfd8cdbcc8a5b3e8ccea9f.jpg", 1);
        if (!frame.empty()) {
            // Convert the OpenCV image (BGR) to a ROS Image message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
            sensor_msgs::msg::Image::SharedPtr image_msg = img_bridge.toImageMsg();
            publisher_->publish(*image_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing image");
        } else {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}

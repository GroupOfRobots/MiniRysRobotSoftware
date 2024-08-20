#include "rclcpp/rclcpp.hpp"
#include "shuttlecock_detect/nodes/DetectorNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detector>());
  rclcpp::shutdown();
  return 0;
}

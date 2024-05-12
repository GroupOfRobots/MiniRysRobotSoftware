#include "rclcpp/rclcpp.hpp"
#include "nodes/CombinedFollowerNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombinedFollower>());
  rclcpp::shutdown();
  return 0;
}

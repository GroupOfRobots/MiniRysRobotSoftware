#include "rclcpp/rclcpp.hpp"
#include "nodes/WallFollowerNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}

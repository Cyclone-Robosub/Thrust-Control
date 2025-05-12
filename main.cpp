#include "thrust_control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thrust_control::ThrustControlNode>());
  rclcpp::shutdown();
  return 0;
}

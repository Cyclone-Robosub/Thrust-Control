#include "thrust_control_node.hpp"

namespace thrust_control
{

ThrustControlNode::ThrustControlNode()
    : Node("thrust_control_node"), 
  supervisor_(this->get_logger(), nullptr)
{
subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
   "array_Cltool_topic", 10, std::bind(&ThrustControlNode::topic_callback, this, std::placeholders::_1));
}

ThrustControlNode::ThrustControlNode(std::unique_ptr<Command_Interpreter_RPi5> interpreter) 
    : Node("thrust_control_node"), 
  supervisor_(this->get_logger(),std::move(interpreter))
{
  subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "array_Cltool_topic", 10, std::bind(&ThrustControlNode::topic_callback, this, std::placeholders::_1));

}

void ThrustControlNode::topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "Received PWM array with %zu values", msg->data.size());
  
  // Print the values in the array
  std::stringstream ss;
  ss << "PWM values: ";
  for (size_t i = 0; i < msg->data.size(); ++i) {
    ss << msg->data[i];
    if (i < msg->data.size() - 1) {
      ss << ", ";
    }
  }
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

}  // namespace thrust_control



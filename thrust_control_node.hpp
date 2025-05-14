#ifndef THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_

#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"      
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

class ThrustControlNode : public rclcpp::Node
{
public:
  
  ThrustControlNode();
  ThrustControlNode(std::unique_ptr<Command_Interpreter_RPi5>);

private:
  
  std_msgs::msg::String::SharedPtr last_message_ = nullptr;
  std::mutex message_mutex_;

  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;
  void duration_callback(const std_msgs::msg::Int64::SharedPtr msg) const;
  void manual_override_callback(const std_msgs::msg::Bool::SharedPtr msg) const;
  void timer_callback();

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr _manual_pwm_subscription;
  rclcpp::TimerBase::SharedPtr timer_;

  ThrustControlSupervisor supervisor_;

};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_

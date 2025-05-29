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
#include "Command.hpp"
#include "command_queue.hpp"
//#include "include/crs_common/crs_ros2_interfaces/src/publish_pwm_cmd.cpp"
// TODO
// Add object for commands
// Add second public function to set pwms & remove setters from step()
// Add functionality for timed commands

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

  void pwm_topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
//  void cpwm_topic_callback(const st
  void duration_callback(const std_msgs::msg::Int64::SharedPtr msg) const;
  void manual_override_callback(const std_msgs::msg::Bool::SharedPtr msg) const;
  void timer_callback();
  
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr _manual_pwm_subscription;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _pwm_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  
  ThrustControlSupervisor supervisor_;
  
  std::string manual_pwm_topic_ = "array_Cltool_topic";
  std::string duration_topic_ = "duation_Cltool_topic";
  std::string position_topic_ = "position_topic";
  std::string waypoint_topic_ = "waypoint_topic";
  std::string sent_pwm_topic_ = "sent_pwm_topic";
    
  pwm_array pwm_ ={ {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}};

};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_ 

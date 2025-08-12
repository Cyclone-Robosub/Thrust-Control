#ifndef THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"      
#include "std_msgs/msg/float64.hpp"  
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command.hpp"
#include "command_queue.hpp"
#include "crs_ros2_interfaces/msg/pwm_cmd.hpp"

//#include "include/crs_common/crs_ros2_interfaces/src/publish_pwm_cmd.cpp"
// TODO: Add position and waypoint callbacks:w


namespace thrust_control
{

class ThrustControlNode : public rclcpp::Node
{
public:
  ThrustControlNode(std::unique_ptr<Command_Interpreter_RPi5>);



  // getters for unit testing
  pwm_array get_user_pwm() const {return user_pwm_;}
  pwm_array get_thruster_pwm() const {return thruster_pwm_;}
  float get_duration() const {return duration_;}
  bool get_manual_override() const {return manual_override_;}
  bool get_is_timed_command() const {return is_timed_command_;}
  ControlMode get_control_mode() const {return control_mode_;}
  Position get_position() const {return position_;}
  bool get_voltageLow() const {return isLowVoltage;}
  Position get_waypoint() const {return waypoint_;}
  std::array<int, 2> get_pwm_limit() const {return pwm_limit_;}

private:
  std_msgs::msg::String::SharedPtr last_message_ = nullptr;
  std::mutex message_mutex_;

  void pwm_topic_callback(const crs_ros2_interfaces::msg::PwmCmd::SharedPtr msg);
  void voltage_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void pwm_limit_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void control_mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void timer_callback();
  void send_pwm();

  rclcpp::Subscription<crs_ros2_interfaces::msg::PwmCmd>::SharedPtr _manual_pwm_subscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _control_mode_subscription;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _position_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _voltage_subscription;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _waypoint_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr _pwm_limit_subscription;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _pwm_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  
  ThrustControlSupervisor supervisor_;
  
  std::string manual_pwm_topic_ = "pwm_cmd_topic";
  std::string position_topic_ = "position_topic";
  std::string waypoint_topic_ = "waypoint_topic";
  std::string sent_pwm_topic_ = "sent_pwm_topic";
  std::string control_mode_topic_ = "control_mode_topic";
  std::string pwm_limit_topic_ = "pwm_limit_topic";

  // start at stop set
  pwm_array user_pwm_ = { {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}};

  // this is the pwm that has last been sent to motors
  pwm_array thruster_pwm_ = { {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}};

  float duration_ = 0;
  bool manual_override_ = false;
  bool is_timed_command_ = false;
  ControlMode control_mode_ = ControlMode::FeedForward;
  std::atomic<bool> isLowVoltage = false;
  std::array<int, 2> pwm_limit_ = {1100, 1900};
  Position position_ = {0,0,0,0,0,0};
  Position waypoint_ = {0,0,0,0,0,0};
};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_ 

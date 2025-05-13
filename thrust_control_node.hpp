#ifndef THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"      
#include "std_msgs/msg/bool.hpp"
#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

class ThrustControlNode : public rclcpp::Node
{
public:
  ThrustControlNode();

private:
  void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const;
  void duration_callback(const std_msgs::msg::Int64::SharedPtr msg) const;
  void manual_override_callback(const std_msgs::msg::Bool::SharedPtr msg) const;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  ThrustControlSupervisor supervisor_;

  bool isManualEnabled = false;
  bool isManualOverride = false;
  bool isRunningThrusterCommand = false;
  bool AllowDurationSync = false;
  std::mutex thruster_mutex;
  std::mutex array_duration_sync_mutex;
  std::mutex Manual_Mutex;

  float angular_velocity_x = NULL_SENSOR_VALUE;
  float angular_velocity_y = NULL_SENSOR_VALUE;
  float angular_velocity_z = NULL_SENSOR_VALUE;
  float linear_acceleration_x = NULL_SENSOR_VALUE;
  float linear_acceleration_y = NULL_SENSOR_VALUE;
  float linear_acceleration_z = NULL_SENSOR_VALUE;

  float mag_field_x = NULL_SENSOR_VALUE;
  float mag_field_y = NULL_SENSOR_VALUE;
  float mag_field_z = NULL_SENSOR_VALUE;

  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr;
  std::vector<PwmPin *> thrusterPins;
  std::vector<DigitalPin *> digitalPins;
  pwm_array our_pwm_array;
  std::queue<std::unique_ptr<Pwm_Command>> ManualPWMQueue;

  pwm_array given_array;

  // The current PWM and duration ptr will and should always have a value regardless of what Executive DecisionLoop or
  // Send Thrusters want. However, SendThrusters can "finish" a current PWM and duration and will say that it wants a
  // new command, but it can be the same current PWM if ExecutiveDecision decides so. Executive Decision (on its own
  // thread) will see that SendThrusters is not running a command and give it a new current PWM. This is made so that
  // Executive Decision has the chance to give PWM a new Command if the current one is a timedPWM. In Later uses, the
  // State file should use the current PWM that the Send Thruster is using.
  std::unique_ptr<Pwm_Command> currentCommand_ptr;
  // bool isQueuePWMEmpty = true;

  //Will REDUCE AND FIX JUST MAKING SURE ALL VARIABLES ARE DECLARED.
  std::ofstream& stateFile;
  std::ostream& output;
  std::ostream& error;
  std::mutex sensor_mutex;
  std::mutex Queue_pwm_mutex;
  std::mutex imu_mutex;
  std::mutex ThrusterCommand_mutex;
  std::mutex Manual_Override_mutex;
  std::mutex command_mutex;
  std::condition_variable SendToDuration_change;
  std::condition_variable PWM_cond_change;
  std::condition_variable Thruster_cond_change;
  std::condition_variable Change_Manual;
  std::string depth_pressure_msg = "Depth Sensor Not Started Yet";
  std::string imu_msg;
  std::vector<float> imu_data;
  float depth = NULL_SENSOR_VALUE;
  float pressure = NULL_SENSOR_VALUE;
  bool loopIsRunning;
  bool tasksCompleted;
  std::string userinput;
  int duration_int_pwm;
  std::string typeOfExecute;

  void replaceCurrentCommand();
  std::string getCurrentDateTime();
  void clearQueue();
  void haltCurrentCommand();

};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_NODE_HPP_

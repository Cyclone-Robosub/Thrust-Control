#ifndef THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_

#include <string>
#include <array>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "Command-Interpreter/lib/Command_Interpreter.h"
#include "command_manager.hpp"

namespace thrust_control
{

class ThrustControlSupervisor
{
public:
	
  explicit ThrustControlSupervisor(
          rclcpp::Logger logger,
          std::unique_ptr<Command_Interpreter_RPi5>);
  void step(	  
	std::string control_mode,
    std::array<int, 8> pwm,
	float duration,
	std::array<float, 6> error,
	std::array<float, 6> waypoint);

private:
  
 void process_pwm_command(std::array<int, 8> pwm_array);
  void feed_forward_pwm(std::array<int, 8> pwm_array);
  void pid_pwm(std::array<int, 8> pwm_array);
 
  static constexpr const char* FEED_FORWARD = "feed-forward";
  static constexpr const char* PID = "pid";
  
  std::string _control_mode;
  std::array<int, 8> _manual_pwm;
  std::array<float, 6> _current_waypoint;
  float _duration;
  std::array<float, 6> _current_position;
  std::array<float, 6> _waypoint;
  
  CommandManager _command_manager;
  rclcpp::Logger _logger;
};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_

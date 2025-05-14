#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
        rclcpp::Logger logger,
        std::unique_ptr<Command_Interpreter_RPi5> interpreter)
  : _logger(logger),
    _interpreter(std::move(interpreter))

{
}

void ThrustControlSupervisor::step(
  std::string control_mode,
  std::array<int, 8> pwm,
  float duration,
  std::array<float, 6> position,
  std::array<float, 6> waypoint)
{
	this->_control_mode = control_mode;
	this->_manual_pwm = pwm;
	this->_duration = duration;
	this->_current_position = position;
	this->_waypoint = waypoint;

	this->process_pwm_command(_manual_pwm);
}


void ThrustControlSupervisor::process_pwm_command(std::array<int, 8> pwm)
{
  if (this->_control_mode == FEED_FORWARD) { this->feed_forward_pwm(pwm); }
  if (this->_control_mode == PID){ this->pid_pwm(pwm); }
}  


void ThrustControlSupervisor::feed_forward_pwm(std::array<int, 8> pwm)  
{
    // tanishqs code goes here
  
}

void ThrustControlSupervisor::pid_pwm(std::array<int, 8> pwm)
{
    // exported code from simulink goes here

}
}


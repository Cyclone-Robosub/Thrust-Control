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
<<<<<<< HEAD
    // tanishqs code goes here
  
=======
  // feed_forward_pwm implementation
  // Placeholder for compile
  pwm[0] = 0;
>>>>>>> 82630cbe66a68703f1f05db4a38c7a708e6495d5
}

void ThrustControlSupervisor::pid_pwm(std::array<int, 8> pwm)
{
<<<<<<< HEAD
    // exported code from simulink goes here

=======
  // pid_pwm implementation
  // Placeholder for compile
  pwm[0] = 0;
>>>>>>> 82630cbe66a68703f1f05db4a38c7a708e6495d5
}
}


#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
        rclcpp::Logger logger,
        std::unique_ptr<Command_Interpreter_RPi5> interpreter)
  : logger_(logger),
    command_manager_(std::move(interpreter))

{
}

void ThrustControlSupervisor::step(
  std::string control_mode,
  std::array<int, 8> pwm,
  float duration,
  std::array<float, 6> position,
  std::array<float, 6> waypoint)
{
	this->control_mode_ = control_mode;
	this->manual_pwm_ = pwm;
	this->duration_ = duration;
	this->current_position_ = position;
	this->waypoint_ = waypoint;

	this->process_pwm_command(manual_pwm_);
}


void ThrustControlSupervisor::process_pwm_command(std::array<int, 8> pwm)
{
  if (this->control_mode_ == FEED_FORWARD) { this->feed_forward_pwm(pwm); }
  if (this->control_mode_ == PID){ this->pid_pwm(pwm); }
}  


void ThrustControlSupervisor::feed_forward_pwm(std::array<int, 8> pwm)  
{
    CommandComponent commandComponent;

  
}

void ThrustControlSupervisor::pid_pwm(std::array<int, 8> pwm)
{

}
}


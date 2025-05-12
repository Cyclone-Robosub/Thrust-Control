#include "thrust_control_supervisor.hpp"


namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
  rclcpp::Logger logger,
  std::shared_ptr<Command_Interpreter_RPi5> command_interpreter
  )
  : logger_(logger), 
    command_interpreter_(command_interpreter)

{
    if (!command_interpreter_) {
    RCLCPP_WARN(logger_, "Command interpreter not provided in constructor");
  }
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
  // process_pwm_command implementation
  if (this->control_mode_ == FEED_FORWARD) { this->feed_forward_pwm(pwm); }
  if (this->control_mode_ == PID){ this->pid_pwm(pwm); }


}  // namespace thrust_control


void ThrustControlSupervisor::feed_forward_pwm(std::array<int, 8> pwm)  
{
  pwm_array pwm_cmd;
  for (int i = 0; i < 8; i++) 
  {
    pwm_cmd.pwm_signals[i] = pwm[i];
  }
  
  if (!command_interpreter_) 
  {
    RCLCPP_ERROR(logger_, "Command interpreter not initialized");
    return;
  }
  
  if (duration_ <= 0.0f)
   {
    Untimed_Command command(pwm_cmd);
    command.execute(*command_interpreter_);
  } 
  else 
  {
    std::chrono::milliseconds duration_ms(static_cast<int>(duration_ * 1000));
    Timed_Command command(pwm_cmd, duration_ms);
    command.execute(*command_interpreter_);
  }


}

void ThrustControlSupervisor::pid_pwm(std::array<int, 8> pwm)
{
  // pid_pwm implementation
  // use simulink exported code
}
}

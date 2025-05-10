#include "thrust_control/thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(rclcpp::Logger logger)
  : logger_(logger)
{
  // Constructor body
}

void ThrustControlSupervisor::step(
  std::string control_mode,
  std::array<int, 8> pwm,
  float duration,
  std::array<float, 6> position,
  std::array<float, 6> waypoint)
{
	this.control_mode = control_mode;
	this.manual_pwm = pwm;
	this.duration = duration;
	this.position = position;
	this.waypoint = waypoint;
	
	this.process_pwm_command(manual_pwm_);
}


void ThrustControlSupervisor::process_pwm_command(pwm)
{
  // process_pwm_command implementation
  if (self.control_mode == FEED_FORWARD) { self.feed_forward_pwm(pwm); }
  if (self.control_mode == PID){ self.pid_pwm(pwm); }


}  // namespace thrust_control
}

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
void ThrustControlSupervisor::set_pwm(
  std::array<int, 8> pwm)
{
  this->_manual_pwm = pwm;
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

	this->process_pwm_command();
}

void ThrustControlSupervisor::process_pwm_command()
{
  RCLCPP_INFO(_logger, "step it\n");
  if (this->_control_mode == FEED_FORWARD) { this->feed_forward_pwm(); }
  if (this->_control_mode == PID){ this->pid_pwm(); }
}  

void ThrustControlSupervisor::feed_forward_pwm()  
{
  // manual implentation
  _auto_flag = false;
  _interpreter->untimed_execute(_manual_pwm);
}

void ThrustControlSupervisor::pid_pwm()
{
    // exported code from simulink goes here
    if ( _auto_flag == false )
    { 
        _auto_flag = true;
        //simulink.initialize()
    }
    //simulink.step()
    
}

void log_array(rclcpp::Logger logger, const std::array<int, 8>& arr) {
    std::stringstream ss;
    ss << "Array values: [";
    for (size_t i = 0; i < arr.size(); ++i) {
        ss << arr[i];
        if (i < arr.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";

    RCLCPP_INFO(logger, "%s", ss.str().c_str());
}
}

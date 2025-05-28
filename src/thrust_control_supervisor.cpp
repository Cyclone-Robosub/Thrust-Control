#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
        rclcpp::Logger logger,
        std::unique_ptr<Command_Interpreter_RPi5> interpreter,
        CommandQueue command_queue)
  : _logger(logger),
    _interpreter(std::move(interpreter)),
    _controller(std::make_unique<controller_codegenTest>()),
    current_command(std::make_unique<Untimed_Command>(stop_set, false))
{
  this->command_queue = command_queue;
  current_command->start();
}

void ThrustControlSupervisor::push_to_pwm_queue(std::unique_ptr<SupervisorCommand> new_command)
{
  if (new_command->isOverride()) {
    current_command = std::move(new_command);
    command_queue = CommandQueue();
  }
  else {
    command_queue.push_command(std::move(new_command));
  }
}

void ThrustControlSupervisor::step(
  ControlMode control_mode,
  std::array<float, 6> position,
  std::array<float, 6> waypoint)
{

	this->_control_mode = control_mode;
	this->_current_position = position;
	this->_waypoint = waypoint;

	this->process_pwm_command();
}

void ThrustControlSupervisor::process_pwm_command()
{
  if (this->_control_mode == FeedForward) { this->feed_forward_pwm(); }
  if (this->_control_mode == PID){ this->pid_pwm(); }
}  

void ThrustControlSupervisor::feed_forward_pwm()  
{
  _auto_flag = false;
  if (current_command->isFinished())
  {
    current_command = command_queue.get_command_from_queue(
            std::move(current_command));
  }
  current_command->start();
  _interpreter->untimed_execute(current_command->getPwms());
}

void ThrustControlSupervisor::pid_pwm()
{
    if ( _auto_flag == false )
    { 
        _auto_flag = true;
        //pathfinder->initialize();
        _controller->initialize();

    }
    //step_pathfinder();
    step_controller();

}

void step_pathfinder()
{
    // give inputs to pathfinder
    // take outputs (refernce singal)
}
void ThrustControlSupervisor::step_controller()
{
    pwm_array new_pwm;
    for (int i = 0; i < 6; i++)   
    {
        _controller->rtU.Input[i] = _current_position[i] - _waypoint[i];
    }
    _controller->step();
    
    for (int i = 0; i < 8; i++)
    {
        new_pwm.pwm_signals[i] = _controller->rtY.Out1[i];
    }
    current_command = std::make_unique<Untimed_Command>(new_pwm);
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

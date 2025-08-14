#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
        rclcpp::Logger logger,
        std::unique_ptr<Command_Interpreter_RPi5> interpreter,
        CommandQueue command_queue)
  : _logger(logger),
    _interpreter(std::move(interpreter)),
    _controller(std::make_unique<PID_Controller>()),
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

void ThrustControlSupervisor::push_to_pwm_queue(pwm_array pwm, float duration, bool is_timed_command, bool is_override)
{
  if (is_timed_command) {
    auto timed_command = std::make_unique<Timed_Command>(pwm, std::chrono::milliseconds(static_cast<int>(duration)), is_override);
    push_to_pwm_queue(std::move(timed_command));
  }
  else {
    auto untimed_command = std::make_unique<Untimed_Command>(pwm, is_override);
    push_to_pwm_queue(std::move(untimed_command));
  }
}
void ThrustControlSupervisor::limit_command(
  std::unique_ptr<SupervisorCommand>& command)
{
  pwm_array pwms = command->getPwms();
  for (int i = 0; i < 8; i++)
  {
    if (pwms.pwm_signals[i] < pwm_limit_[0])
    {
      pwms.pwm_signals[i] = pwm_limit_[0];
    }
    if (pwms.pwm_signals[i] > pwm_limit_[1])
    {
      pwms.pwm_signals[i] = pwm_limit_[1];
    }
  }
  command->setPwms(pwms);
}

void ThrustControlSupervisor::step(
  ControlMode control_mode,
  Position position,
  Position waypoint)
{

	this->_control_mode = control_mode;
	this->_current_position = position;
	this->_waypoint = waypoint;

	this->process_pwm_command();
}

void ThrustControlSupervisor::process_pwm_command()
{
  if(isLowVoltageReading){
    pwm_array stop_pwm = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    current_command = std::make_unique<Untimed_Command>(stop_pwm);
  }else{
  if (this->_control_mode == FeedForward) { this->feed_forward_pwm(); }
  if (this->_control_mode == PID){ this->pid_pwm(); }
  }
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
  limit_command(current_command);
  _interpreter->untimed_execute(current_command->getPwms());
}

void ThrustControlSupervisor::pid_pwm()
{
    if ( _auto_flag == false )
    { 
        std::cout << "initializing controller\n";
        _auto_flag = true;
        _controller = std::make_unique<PID_Controller>();
        _controller->initialize();
        std::vector<std::vector<double>> pid_values = 
        {
          {18.568,0.0,0.0,17.247},
          {18.568,0.0,0.0,17.247},
          {18.568,0.0,0.0,17.247},
          {4.773,0.0,0.0,54.54},
          {6.912,0.0,0.0,54.54},
          {6.782,0.0,0.0,114.828}
        };
        PID_Controller::ExtU_PID_Controller_T extU;
        extU.controller_mode = 0.0;
        extU.DFC_error[0] = 0.0;
        extU.DFC_error[1] = 0.0;

        for (int i = 0; i < 4; i++) {
          extU.PIDN_X[i] = pid_values[0][i];
          extU.PIDN_Y[i] = pid_values[1][i];
          extU.PIDN_Z[i] = pid_values[2][i];
          extU.PIDN_roll[i] = pid_values[3][i];
          extU.PIDN_pitch[i] = pid_values[4][i];
          extU.PIDN_yaw[i] = pid_values[5][i];
        }
        _controller->setExternalInputs(&extU);
    }
    step_controller();
    limit_command(current_command);
    _interpreter->untimed_execute(current_command->getPwms());
}

void ThrustControlSupervisor::step_controller()
{
    pwm_array new_pwm;
    PID_Controller::ExtU_PID_Controller_T extU;
    PID_Controller::ExtY_PID_Controller_T extY;

    for (int i = 0; i < 6; i++)   
    {
      extU.states[i] = _current_position[i];
      extU.waypoint[i] = _waypoint[i];
    }

    // correct degrees to radians for angular errors
    for (int i = 3; i < 6; i++)
    { 
        extU.states[i] = extU.states[i] * M_PI / 180.0;
        extU.waypoint[i] = extU.waypoint[i] * M_PI / 180.0;
    }
    _controller->setExternalInputs(&extU);
    _controller->step();

    extY = _controller->getExternalOutputs();
    for (int i = 0; i < 8; i++)
    {
        new_pwm.pwm_signals[i] = extY.PWM[i];
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

#include "thrust_control_supervisor.hpp"

namespace thrust_control
{

ThrustControlSupervisor::ThrustControlSupervisor(
        rclcpp::Logger logger,
        std::unique_ptr<Command_Interpreter_RPi5> interpreter)
  : _logger(logger),
    _command_manager(std::move(interpreter))

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
void ThrustControlSupervisor::ControlLoop(){
  while (loopIsRunning) {
    std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
    if (isManualEnabled) {
      std::unique_lock<std::mutex> Manual_Override_Lock(Manual_Override_mutex);
      if (isManualOverride) {
        //if the sendThrusterCommand is currently running or has a task.
        if (isRunningThrusterCommand) {
          output << "manual override" << std::endl;
          haltCurrentCommand();
          replaceCurrentCommand();
          //override was successful; revert back.
          isManualOverride = false;
        }
      }
      typeOfExecute = "timed_execute";
      std::unique_lock<std::mutex> QueuepwmValuesLock(Queue_pwm_mutex,
                                                      std::defer_lock);
      PWM_cond_change.wait(QueuepwmValuesLock,
                            [this] { return !(ManualPWMQueue.size() == 0); });
      // Add comment here below and above.
      if (!isRunningThrusterCommand) {
        replaceCurrentCommand();
      }else{
        
      }
    }
  }
  // Need to see William's python code to move foward.
}
void ThrusterControlSupervisor::sendThrusterCommand(){
  while (loopIsRunning) {
    if (isRunningThrusterCommand) {
      output << "Send Thruster Command is doing its job" << std::endl;
      std::unique_lock<std::mutex> current_command_lock(
        command_mutex);
      currentCommand_ptr->execute(*commandInterpreter_ptr);
      std::unique_lock<std::mutex> statusThruster(thruster_mutex);
      isRunningThrusterCommand = false;
    }
    output << "Finished Thruster Command\n" << std::endl;
  }
}

}


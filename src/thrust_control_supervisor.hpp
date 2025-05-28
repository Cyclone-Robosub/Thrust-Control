#ifndef THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_

#include <string>
#include <array>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "Command_Interpreter.hpp"
#include "command_queue.hpp"

// Temporarily disable unused-function warnings for this include
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "../include/exported-PID-control/src/controller_codegenTest.h"
#pragma GCC diagnostic pop

// TODO
// implement command queue for manual control
// implement waypoint list
// implement function that both sends pwms to comman_interpreter and posts to topic
// replace std::array<int, 8> with pwm::array from command_interpreter
// remove ROS dependencies from this class

namespace thrust_control
{

enum ControlMode {Auto, FeedForward, PID};

class ThrustControlSupervisor
{
public:
	
  explicit ThrustControlSupervisor(
          rclcpp::Logger logger,
          std::unique_ptr<Command_Interpreter_RPi5>,
          CommandQueue command_queue);
  
  void update_pwm_queue(std::unique_ptr<SupervisorCommand> new_command);

  void step(
	  ControlMode control_mode,
	  std::array<float, 6> position,
	  std::array<float, 6> waypoint);
  
  pwm_array get_current_command_pwm(){ return current_command->getPwms();}
  ControlMode get_control_mode() {return _control_mode;}
  CommandQueue get_command_queue() {return command_queue;}
  std::array<float, 6> get_current_position() {return _current_position;}
  std::array<float, 6> get_waypoint() {return _waypoint;}

private:
  
  void process_pwm_command();
  void feed_forward_pwm();
  void pid_pwm(); 
  void step_controller();
  void execute_pwm(std::array<int, 8>);

  bool _auto_flag = false;

  rclcpp::Logger _logger;
  std::unique_ptr<Command_Interpreter_RPi5> _interpreter;
  std::unique_ptr<controller_codegenTest> _controller;
  
  CommandQueue command_queue;
  std::unique_ptr<SupervisorCommand> current_command;
  ControlMode _control_mode;
  ControlMode _last_control_mode;

  // waypoint is given by executive control loop, longer distance
  std::array<float, 6> _current_waypoint;

  // reference position is generated each step iteration by the
  // trajectory generator
  std::array<float, 6> _current_reference_position;
  std::array<float, 6> _current_position;
  std::array<float, 6> _waypoint;
  pwm_array _current_pwm = {{1500,1500,1500,1500,1500,1500,1500, 1500}};

};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_ 

#include "thrust_control_node.hpp"
#include "include/Command_Interpreter/src/Command_Interpreter.hpp"
#include "command_interpreter_pointer.cpp"



int main(int argc, char * argv[])
{
  std::ostream& output = std::cout;
  std::ostream& error = std::cerr;
  std::ostream& log_file = std::cout; //replace with actual log file probably

  auto commandInterpreter_ptr = make_command_interpreter_ptr(output, error, log_file);

  rclcpp::init(argc, argv);
  
  rclcpp::spin(
          std::make_shared<thrust_control::ThrustControlNode>(
              std::move(commandInterpreter_ptr)));
  
  rclcpp::shutdown();
  return 0;
}

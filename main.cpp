#include "thrust_control_node.hpp"
#include "Command-Interpreter/lib/Command_Interpreter.h"


std::unique_ptr<Command_Interpreter_RPi5> makeCommandInterpreterPtr
(
        std::ostream& output, 
        std::ostream& error, 
        std::ostream& logFile) 
{
    auto PhysicalPins = std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9};
      std::vector<PwmPin *> thrusterPins;

      for (auto i : PhysicalPins) 
      {
        thrusterPins.push_back(new HardwarePwmPin(i, logFile, output, error));
      }
  
      auto wiringControl = WiringControl(output, logFile, error);
      std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr = 
          std::make_unique<Command_Interpreter_RPi5>(
          thrusterPins, 
          std::vector<DigitalPin *>{}, 
          wiringControl, 
          logFile, 
          output, 
          error);
  
      return commandInterpreter_ptr;
    }
int main(int argc, char * argv[])
{
  std::ostream& output = std::cout;
  std::ostream& error = std::cerr;
  std::ostream& log_file = std::cout; //replace with actual log file probably

  auto commandInterpreter_ptr = makeCommandInterpreterPtr(output, error, log_file);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thrust_control::ThrustControlNode>());
  rclcpp::shutdown();
  return 0;
}

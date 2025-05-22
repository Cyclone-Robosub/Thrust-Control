#include <memory>
#include "command_interpreter_pointer.hpp"

std::unique_ptr<Command_Interpreter_RPi5> make_command_interpreter_ptr
(
    std::ostream& output, 
    std::ostream& error, 
    std::ostream& logFile
)
{
    auto PhysicalPins = std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9};
      std::vector<PwmPin *> thrusterPins;

      for (auto i : PhysicalPins) 
      {
        thrusterPins.push_back(new HardwarePwmPin(i, output, logFile, error));
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

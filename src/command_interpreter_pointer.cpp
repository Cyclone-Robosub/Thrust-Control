#include <memory>
#include "command_interpreter_pointer.hpp"

std::unique_ptr<Command_Interpreter_RPi5> make_command_interpreter_ptr (
                                          std::ostream& output, 
                                          std::ostream& logFile,
                                          std::ostream& error) {
  auto PhysicalPins = std::vector<int>{8, 9, 6, 7, 13, 11, 12, 10};
  std::vector<PwmPin *> thrusterPins;

  for (auto i : PhysicalPins) {
    thrusterPins.push_back(new PwmPin(i, output, logFile, error));
  }

  auto wiringControl = WiringControl(output, logFile, error);
  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr = 
      std::make_unique<Command_Interpreter_RPi5>(
        thrusterPins, 
        std::vector<DigitalPin *>{}, 
        wiringControl, 
        output,
        logFile, 
        error);
    
  commandInterpreter_ptr->initializePins();

  return commandInterpreter_ptr;
} 

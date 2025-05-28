#pragma once

#include <memory>
#include "Command_Interpreter.hpp"

// Function to create and return a Command_Interpreter_RPi5 object
std::unique_ptr<Command_Interpreter_RPi5> make_command_interpreter_ptr(
    std::ostream& output,
    std::ostream& logFile,
    std::ostream& error
); 

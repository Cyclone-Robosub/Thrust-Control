// CommandManager.hpp
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Command-Interpreter/lib/Command_Interpreter.h"
#include "Pwm_Command.hpp"

class CommandManager {
public:
    
    CommandManager(
        std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr,
        std::ostream& output = std::cout,
        std::ostream& error = std::cerr,
        std::ostream& logFile = std::cout //add actual log file eventually
    );
    void execute_decision_loop();
    void send_thruster_command();
    
private:
    
    std::unique_ptr<Command_Interpreter_RPi5> _commandInterpreter_ptr;
    std::queue<std::unique_ptr<Pwm_Command>> commandQueue;
    std::unique_ptr<Pwm_Command> currentCommand;
    
    std::ostream& _output;
    std::ostream& _error;
    std::ostream& _logFile;
    
};

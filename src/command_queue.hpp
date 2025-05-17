#pragma once

#include "supervisor_command.hpp"
#include "../Command-Interpreter/lib/Command_Interpreter.h"
#include <queue>
#include <memory>

namespace thrust_control {

class CommandQueue {
public:
    CommandQueue();
    
    void add_command(std::unique_ptr<Command> new_command);
    void add_command(
        const pwm_array& pwm, 
        std::chrono::milliseconds duration,
        bool is_timed,
        bool is_override = false);
    
    std::unique_ptr<Command> make_new_command(
        const pwm_array& pwm, 
        std::chrono::milliseconds duration,
        bool is_timed,
        bool is_override = false);
    
    std::unique_ptr<Command> get_command_from_queue(Command* current_command);
    


private:
    std::queue<std::unique_ptr<Command>> command_queue_;
    pwm_array stop_set_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 }; 
};

}  // namespace thrust_control



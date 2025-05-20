#pragma once

#include "supervisor_command.hpp"
#include "../include/Command_Interpreter/src/Command.hpp"
#include <queue>
#include <memory>

namespace thrust_control {

class CommandQueue {
public:
    CommandQueue();
    
    void push_command(std::unique_ptr<SupervisorCommand> new_command);
    
    static std::unique_ptr<SupervisorCommand> make_new_command(
        const pwm_array& pwm, 
        std::chrono::milliseconds duration,
        bool is_timed,
        bool is_override);
    
    std::unique_ptr<SupervisorCommand> get_command_from_queue(
            std::unique_ptr<SupervisorCommand> current_command);

    private:
    std::queue<std::unique_ptr<SupervisorCommand>> command_queue_;
    pwm_array stop_set_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; 
};

}  // namespace thrust_control



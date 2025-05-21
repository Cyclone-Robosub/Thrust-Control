#pragma once

#include "supervisor_command.hpp"
#include "Command.hpp"
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
    
    CommandQueue& operator=(const CommandQueue& new_command_queue);

    private:
    std::queue<std::unique_ptr<SupervisorCommand>> command_queue_;
};

}  // namespace thrust_control



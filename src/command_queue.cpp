#include "command_queue.hpp"

namespace thrust_control {

CommandQueue::CommandQueue() 
{

}


void CommandQueue::push_command(std::unique_ptr<SupervisorCommand> new_command)
{
    command_queue_.push(std::move(new_command));
}

std::unique_ptr<SupervisorCommand> CommandQueue::make_new_command(
    const pwm_array& pwm, 
    std::chrono::milliseconds duration,
    bool is_timed,
    bool is_override) {
        if (is_timed) {
            return std::move(std::make_unique<Timed_Command>(pwm, duration, is_override));
        }
        else {
            return std::move(std::make_unique<Untimed_Command>(pwm, is_override));
        }
    }

std::unique_ptr<SupervisorCommand> CommandQueue::get_command_from_queue(
    std::unique_ptr<SupervisorCommand> current_command)
{
        if (command_queue_.empty()) 
        {
        return std::make_unique<Untimed_Command>(
                        current_command->onExpirePwm(), 
                        current_command->isOverride());
        }
        else {
            auto next_command = std::move(command_queue_.front());
            command_queue_.pop();
            return next_command;
        }
}
}

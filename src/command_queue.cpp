#include "command_queue.hpp"

namespace thrust_control {

CommandQueue::CommandQueue() {
    command_queue_ = std::deque<std::unique_ptr<SupervisorCommand>>{};
}


void CommandQueue::push_command(std::unique_ptr<SupervisorCommand> new_command)
{
    command_queue_.push_back(std::move(new_command));
}

std::unique_ptr<SupervisorCommand> CommandQueue::make_new_command(
    const pwm_array& pwm, 
    bool is_timed,
    bool is_override,
    std::chrono::milliseconds duration) {
        if (is_timed) {
            return std::move(std::make_unique<Timed_Command>(pwm, duration, is_override));
        }
        else {
            return std::move(std::make_unique<Untimed_Command>(pwm, is_override));
        }
    }

// This needs a parameter current_command so that the commands can persist correctly. For
// example, if the queue is empty, an untimed command should be replaced with itself so that
// the command runs indefinitely. For this, we need to know what the command that we just ran is.
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
            command_queue_.pop_front();
            return next_command;
        }
}

CommandQueue& CommandQueue::operator=(const CommandQueue& new_command_queue) {
    if (this != &new_command_queue) {
        command_queue_.clear();
        for (const auto& command : new_command_queue.command_queue_) {
            command_queue_.push_back(command->clone());
        }
    }

    return *this;
}

CommandQueue::CommandQueue(const CommandQueue& other) {
    command_queue_.clear();
    for (const auto& command : other.command_queue_) {
        command_queue_.push_back(command->clone());
    }
}


}

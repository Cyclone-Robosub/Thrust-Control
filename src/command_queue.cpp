#include "command_queue.hpp"

namespace thrust_control 
{

CommandQueue::CommandQueue() {}


void CommandQueue::add_command(std::unique_ptr<Command> new_command)
{
    command_queue_.push(std::move(new_command));
}

void CommandQueue::add_command(
    const pwm_array& pwm, 
    std::chrono::milliseconds duration,
    bool is_timed,
    bool is_override = false)
{
    auto new_command = make_new_command(pwm, duration, is_timed, is_override);
    command_queue_.push(std::move(new_command));

}

std::unique_ptr<Command> CommandQueue::make_new_command(
    const pwm_array& pwm, 
    std::chrono::milliseconds duration,
    bool is_timed,
    bool is_override = false)
{   

}

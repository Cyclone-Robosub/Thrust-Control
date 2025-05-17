#include "supervisor_command.hpp"

namespace thrust_control
{

SupervisorCommand::SupervisorCommand(
    const pwm_array& pwm, 
    bool is_override = true) :
    pwm_(pwm), 
    duration_(-1), 
    end_time_(-1), 
    is_override_(is_override),
    is_timed_(false) {}
    

SupervisorCommand::SupervisorCommand(
    const pwm_array& pwm, 
    std::chrono::milliseconds duration, 
    bool is_override = true) :
    pwm_(pwm), 
    duration_(duration), 
    end_time_(-1), 
    is_override_(is_override),
    is_timed_(true){}

void SupervisorCommand::start()
{
    if (is_timed_)
    {
        auto now = std::chrono::steady_clock::now();
        end_time_ = now + duration_;
    }
    else { end_time_ = 0; }
}

bool SupervisorCommand::isFinished()
{
    if (is_timed_)
    {
        return (std::chrono::steady_clock::now() >= end_time_);

    }
    else { return false; }

}

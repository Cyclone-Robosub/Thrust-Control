#include "supervisor_command.hpp"

namespace thrust_control {

SupervisorCommand::SupervisorCommand(
    const pwm_array& pwm, 
    bool is_override = true) :
    pwm_(pwm), 
    is_override_(is_override) {}



Untimed_Command::Untimed_Command(const pwm_array& pwm, bool is_override = true) :
         SupervisorCommand(pwm, is_override), is_executed_(false) {}

void Untimed_Command::start() {
    is_executed_ = true;
}

bool Untimed_Command::isFinished() {
    return is_executed_;
}

pwm_array Untimed_Command::onExpirePwm() {
    return pwm_;
}


Timed_Command::Timed_Command(const pwm_array& pwm, std::chrono::milliseconds duration, bool is_override = false) : 
        SupervisorCommand(pwm, is_override), duration_(duration), is_started_(false), end_time_(std::chrono::steady_clock::now()) {}

void Timed_Command::start() {
    if (!is_started_) {
        end_time_ = std::chrono::steady_clock::now() + duration_;
        is_started_ = true;
    }
}

bool Timed_Command::isFinished() {
    return std::chrono::steady_clock::now() >= end_time_;
}

pwm_array Timed_Command::onExpirePwm() {
    return stop_set_;
}

}

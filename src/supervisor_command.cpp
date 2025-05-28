#include "supervisor_command.hpp"
#include "sets.hpp"

namespace thrust_control {

SupervisorCommand::SupervisorCommand(const pwm_array& pwm, bool is_override)
    : pwm_(pwm), is_override_(is_override) {}

Untimed_Command::Untimed_Command(const pwm_array& pwm, bool override)
    : SupervisorCommand(pwm, override), is_executed_(false) {}

std::unique_ptr<SupervisorCommand> Untimed_Command::clone() const {
    return std::make_unique<Untimed_Command>(pwm_, is_override_);
}

void Untimed_Command::start() {
    is_executed_ = true;
}

bool Untimed_Command::isFinished() {
    return is_executed_;
}

pwm_array Untimed_Command::onExpirePwm() {
    return pwm_;
}

Timed_Command::Timed_Command(const pwm_array& pwm, 
                           std::chrono::milliseconds duration,
                           bool override)
    : SupervisorCommand(pwm, override)
    , duration_(duration)
    , is_started_(false) {}

std::unique_ptr<SupervisorCommand> Timed_Command::clone() const {
    return std::make_unique<Timed_Command>(pwm_, duration_, is_override_);
}
    

void Timed_Command::start() {
    if (!is_started_) {
        end_time_ = std::chrono::steady_clock::now() + duration_;
        is_started_ = true;
    }
}

bool Timed_Command::isFinished() {
    if (!is_started_) return false;
    return std::chrono::steady_clock::now() >= end_time_;
}

pwm_array Timed_Command::onExpirePwm() {
    return stop_set;
}

} // namespace thrust_control

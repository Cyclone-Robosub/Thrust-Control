#ifndef COMMAND_SYSTEM_HPP
#define COMMAND_SYSTEM_HPP

#include <array>
#include <chrono>
#include <queue>
#include <memory>
#include <chrono>
#include "../include/Command_Interpreter/src/Command.hpp"
//TODO

namespace thrust_control {

class SupervisorCommand {
public:
    
    SupervisorCommand(const pwm_array& pwm, bool is_override = true);
    SupervisorCommand(
        const pwm_array& pwm, 
        std::chrono::milliseconds duration, 
        bool is_override = true);
    ~SupervisorCommand() = default;
    
    void start();
    bool isFinished();
    pwm_array getPwms() const;
    bool has_override() const;

private:
    pwm_array pwm_;
    std::chrono::milliseconds duration_;
    std::chrono::milliseconds end_time_;
    bool is_override_;
    bool is_timed_;
    
};
//
//
//class Untimed_Command : public Command {
//public:
//    Untimed_Command(const pwm_array& pwm, bool override = true);
//    
//    void start() override;
//    bool isFinished() override;
//    pwm_array onExpirePwm() override;
//
//private:
//    bool is_executed_;
//};
//
//class Timed_Command : public Command {
//public:
//    Timed_Command(const pwm_array& pwm, 
//                  std::chrono::milliseconds duration,
//                  bool override = false);
//    
//    void start() override;
//    bool isFinished() override;
//    pwm_array onExpirePwm() override;
//
//private:
//    std::chrono::milliseconds duration_;
//    std::chrono::time_point<std::chrono::steady_clock> end_time_;
//    bool is_started_;
//    pwm_array stop_set_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // Default stop values
//};
}  // namespace thrust_control

#endif // COMMAND_SYSTEM_HPP 

#ifndef COMMAND_SYSTEM_HPP
#define COMMAND_SYSTEM_HPP

#include <array>
#include <chrono>
#include <queue>
#include <memory>
#include <chrono>
#include "Command_Interpreter/src/Command.hpp"
//TODO

namespace thrust_control {

class SupervisorCommand {
public:
    SupervisorCommand(const pwm_array& pwm, bool is_override = true);
    virtual ~SupervisorCommand() = default;
    
    virtual void start() = 0;
    virtual bool isFinished() = 0;
    virtual pwm_array getPwms() const {return pwm_;};
    virtual bool isOverride() const {return is_override_;};
    virtual pwm_array onExpirePwm() = 0;

protected:
    pwm_array pwm_;
    bool is_override_; 
};

class Untimed_Command : public SupervisorCommand {
public:
   Untimed_Command(const pwm_array& pwm, bool override = true);
   
   void start() override;
   bool isFinished() override;
   pwm_array onExpirePwm() override;

protected:
   bool is_executed_;
};

class Timed_Command : public SupervisorCommand {
public:
   Timed_Command(const pwm_array& pwm, 
                 std::chrono::milliseconds duration,
                 bool override = false);
   
   void start() override;
   bool isFinished() override;
   pwm_array onExpirePwm() override;

protected:
   std::chrono::milliseconds duration_;
   std::chrono::time_point<std::chrono::steady_clock> end_time_;
   bool is_started_;
   pwm_array stop_set_ = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // Default stop values
};
}  // namespace thrust_control

#endif // COMMAND_SYSTEM_HPP 

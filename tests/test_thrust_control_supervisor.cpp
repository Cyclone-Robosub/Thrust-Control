#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "Command.hpp"


class ThrustControlSupervisorTest : public ::testing::Test {
protected:

    ThrustControlSupervisorTest() : logger(rclcpp::get_logger("test_logger")) {}
    
    void SetUp() override 
    {
        if (!rclcpp::ok()) {  rclcpp::init(0, nullptr);}
        auto interpreter = make_command_interpreter_ptr(
                std::cout, 
                std::cout, 
                std::cout);
   }

    void TearDown() override 
    {
        if (rclcpp::ok()) { rclcpp::shutdown(); }
    }

    rclcpp::Logger logger;
};


// Test 1: Initialization
TEST_F(ThrustControlSupervisorTest, CanBeInitialized) {
    // Arrange
         auto test_interpreter = make_command_interpreter_ptr(
                std::cout, 
                std::cout, 
                std::cout);

   
    // Act - this tests if we can initialize the supervisor without exceptions
    ASSERT_NO_THROW({
        thrust_control::ThrustControlSupervisor test_supervisor(
                logger, 
                std::move(test_interpreter),
                thrust_control::CommandQueue());
    });
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorPidNoError) {

    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);
    
    thrust_control::ThrustControlSupervisor supervisor(
            logger, 
            std::move(interpreter),
            thrust_control::CommandQueue());

    thrust_control::ControlMode control_mode = thrust_control::ControlMode::PID;
    std::array<float, 6> position = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> waypoint = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    ASSERT_NO_THROW({
        supervisor.step(control_mode, position, waypoint);
    });
    
    pwm_array current_pwm = supervisor.get_current_pwm();


    bool has_force = false;
    for (int i = 0; i < 8; i++) {
        if (current_pwm.pwm_signals[i] != 1500) {
            has_force = true;
        }
    }
    EXPECT_FALSE(has_force) << "PWM signal should not have a force";
}
TEST_F(ThrustControlSupervisorTest, StepSupervisorPidWithError) {
    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);

    thrust_control::ThrustControlSupervisor supervisor(
            logger, 
            std::move(interpreter),
            thrust_control::CommandQueue());

    thrust_control::ControlMode control_mode = thrust_control::ControlMode::PID;
    std::array<float, 6> position = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 6> waypoint = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    
    ASSERT_NO_THROW({
        supervisor.step(control_mode, position, waypoint);
    });

    pwm_array current_pwm = supervisor.get_current_pwm();

    bool has_force = false;
    for (int i = 0; i < 8; i++) {
        if (current_pwm.pwm_signals[i] != 1500) {
            has_force = true;
        }
    }
    EXPECT_TRUE(has_force) << "PWM signal should have a force";
}   

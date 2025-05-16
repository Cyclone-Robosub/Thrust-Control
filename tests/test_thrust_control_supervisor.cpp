#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "../src/thrust_control_supervisor.hpp"
#include "../Command-Interpreter/lib/Command_Interpreter.h"

// Helper function to create a Command_Interpreter_RPi5 instance for testing
std::unique_ptr<Command_Interpreter_RPi5> createCommandInterpreter() {
    // Create pins for the interpreter
    std::vector<PwmPin*> thrusterPins;
    for (int i = 2; i <= 9; i++) {
        thrusterPins.push_back(new HardwarePwmPin(i, std::cout, std::cout, std::cerr));
    }
    
    // Create the wiring control
    WiringControl wiringControl(std::cout, std::cout, std::cerr);
    
    // Create and return the command interpreter
    return std::make_unique<Command_Interpreter_RPi5>(
        thrusterPins,
        std::vector<DigitalPin*>{},
        wiringControl,
        std::cout,  // log file
        std::cout,  // output
        std::cerr   // error
    );
}

class ThrustControlSupervisorTest : public ::testing::Test {
protected:
    ThrustControlSupervisorTest() 
        : logger(rclcpp::get_logger("test_logger")) {
    }

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    rclcpp::Logger logger;
};

TEST_F(ThrustControlSupervisorTest, CanBeInitialized) {
    // Arrange
    auto interpreter = createCommandInterpreter();
    
    // Act - this tests if we can initialize the supervisor without exceptions
    ASSERT_NO_THROW({
        thrust_control::ThrustControlSupervisor supervisor(logger, std::move(interpreter));
    });
}

// Test setting PWM values
TEST_F(ThrustControlSupervisorTest, CanSetPwmValues) {
    // Arrange
    auto interpreter = createCommandInterpreter();
    thrust_control::ThrustControlSupervisor supervisor(logger, std::move(interpreter));
    
    // Act & Assert - test if setting PWM values doesn't throw
    std::array<int, 8> test_pwm = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    ASSERT_NO_THROW({
        supervisor.set_pwm(test_pwm);
    });
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
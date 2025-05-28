#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"

using namespace thrust_control;

class ThrustControlSupervisorTest : public ::testing::Test {
protected:

    ThrustControlSupervisorTest() : logger(rclcpp::get_logger("test_logger")) {}
    
    void SetUp() override 
    {
        if (!rclcpp::ok()) {  rclcpp::init(0, nullptr);}
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
    auto interpreter = make_command_interpreter_ptr(std::cout, std::cout, std::cout);
    
    // Act - this tests if we can initialize the supervisor without exceptions
    ASSERT_NO_THROW({
        thrust_control::ThrustControlSupervisor supervisor(
                logger, 
                std::move(interpreter),
                thrust_control::CommandQueue());
    });
}


TEST_F(ThrustControlSupervisorTest, StepSupervisor) {

    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);

    thrust_control::ThrustControlSupervisor supervisor(
            logger, 
            std::move(interpreter),
            thrust_control::CommandQueue());
    
    supervisor.step(Auto, {0,1.5,1.8,-99,0,0}, {-9,10,42.5,42.0,0,1});
    // ASSERT_EQ(supervisor.get_control_mode(), Auto);
    // ASSERT_EQ(supervisor.get_current_position(), ({0,1.5,1.8,-99,0,0}));
    // ASSERT_EQ(supervisor.get_waypoint(), ({-9,10,42.5,42.0,0,1}));
}
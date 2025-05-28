#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "sets.hpp"
#include <fstream>

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
    std::ofstream nullOut = std::ofstream("/dev/null");
};


// Test 1: Initialization
TEST_F(ThrustControlSupervisorTest, CanBeInitialized) {
    // Arrange
    auto interpreter = make_command_interpreter_ptr(nullOut, nullOut, nullOut);
    
    // Act - this tests if we can initialize the supervisor without exceptions
    ASSERT_NO_THROW({
        thrust_control::ThrustControlSupervisor supervisor(
                logger, 
                std::move(interpreter),
                thrust_control::CommandQueue());
    });
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorAuto) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    
    supervisor.step(Auto, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), Auto);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorCustomFeedForward) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    pwm_array pwm = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(pwm);

    supervisor.push_to_pwm_queue(std::move(untimed_command));
    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm);
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorMultiStep) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};

    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    std::unique_ptr<SupervisorCommand> command_1 = std::make_unique<Untimed_Command>(pwm_1, false);
    std::unique_ptr<SupervisorCommand> command_2 =  std::make_unique<Untimed_Command>(pwm_2, false);

    supervisor.push_to_pwm_queue(std::move(command_1));
    supervisor.push_to_pwm_queue(std::move(command_2));

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_1);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorUntimedOverrideUntimed) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    std::unique_ptr<SupervisorCommand> command_1 = std::make_unique<Untimed_Command>(pwm_1, false);
    std::unique_ptr<SupervisorCommand> command_2 =  std::make_unique<Untimed_Command>(pwm_2);

    supervisor.push_to_pwm_queue(std::move(command_1));
    supervisor.push_to_pwm_queue(std::move(command_2));

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorUntimedOverrideTimed) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    std::unique_ptr<SupervisorCommand> command_1 = std::make_unique<Timed_Command>(pwm_1, std::chrono::milliseconds(50));
    std::unique_ptr<SupervisorCommand> command_2 =  std::make_unique<Untimed_Command>(pwm_2);

    supervisor.push_to_pwm_queue(std::move(command_1));
    supervisor.push_to_pwm_queue(std::move(command_2));

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorTimedOverrideTimed) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    std::unique_ptr<SupervisorCommand> command_1 = std::make_unique<Timed_Command>(pwm_1, std::chrono::milliseconds(50));
    std::unique_ptr<SupervisorCommand> command_2 =  std::make_unique<Timed_Command>(pwm_2, std::chrono::milliseconds(50), true);

    supervisor.push_to_pwm_queue(std::move(command_1));
    supervisor.push_to_pwm_queue(std::move(command_2));

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorTimedOverrideUntimed) {

    auto interpreter = make_command_interpreter_ptr(
        nullOut, 
        nullOut, 
        std::cerr);

    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    std::array<float,6> position = {0,1.5,1.8,-99,0,0};
    std::array<float,6> waypoints = {-9,10,42.5,42.0,0,1};
    
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    std::unique_ptr<SupervisorCommand> command_1 = std::make_unique<Untimed_Command>(pwm_1, false);
    std::unique_ptr<SupervisorCommand> command_2 =  std::make_unique<Timed_Command>(pwm_2, std::chrono::milliseconds(50), true);

    supervisor.push_to_pwm_queue(std::move(command_1));
    supervisor.push_to_pwm_queue(std::move(command_2));

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_command_pwm(), pwm_2);
}
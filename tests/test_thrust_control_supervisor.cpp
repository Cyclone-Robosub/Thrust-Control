#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "Command.hpp"
#include "position.hpp"
#include "sets.hpp"
#include <fstream>

using namespace thrust_control;

class ThrustControlSupervisorTest : public ::testing::Test {
protected:

    ThrustControlSupervisorTest() : logger(rclcpp::get_logger("test_logger")) {}
    std::ofstream nullOut = std::ofstream("/dev/null");
    std::unique_ptr<Command_Interpreter_RPi5> interpreter;
    rclcpp::Logger logger;
    
    void SetUp() override {
        if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
        interpreter = make_command_interpreter_ptr(
                nullOut, 
                nullOut, 
                std::cerr);
        interpreter->setAllPwmLimits(1100, 1900);
   }

    void TearDown() override {
        if (rclcpp::ok()) { rclcpp::shutdown(); }
    }

};


// Test 1: Initialization
TEST_F(ThrustControlSupervisorTest, CanBeInitialized) {
    // Arrange
         auto test_interpreter = make_command_interpreter_ptr(
                nullOut, 
                nullOut, 
                std::cerr);
    
    // Act - this tests if we can initialize the supervisor without exceptions
    ASSERT_NO_THROW({
        thrust_control::ThrustControlSupervisor test_supervisor(
                logger, 
                std::move(test_interpreter),
                thrust_control::CommandQueue());
    });
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorPidNoError) {    
    thrust_control::ThrustControlSupervisor supervisor(
            logger, 
            std::move(interpreter),
            thrust_control::CommandQueue());

    thrust_control::ControlMode control_mode = thrust_control::ControlMode::PID;
    Position position(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Position waypoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    
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
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // Default constructor creates (0,0,0,0,0,0)
    Position waypoints(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
   
 
    supervisor.step(PID, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), PID);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);

    pwm_array current_pwm = supervisor.get_current_pwm();

    bool has_force = false;
    for (int i = 0; i < 8; i++) {
        if (current_pwm.pwm_signals[i] != 1500) {
            has_force = true;
        }
    }
    EXPECT_TRUE(has_force) << "PWM signal should have a force";
}   

TEST_F(ThrustControlSupervisorTest, StepSupervisorCustomFeedForward) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);
 
    pwm_array pwm = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(pwm);

    supervisor.push_to_pwm_queue(std::move(untimed_command));
    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm);
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorMultiStep) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);

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
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_1);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorUntimedOverrideUntimed) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);
    
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
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorUntimedOverrideTimed) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);
    
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
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}


TEST_F(ThrustControlSupervisorTest, StepSupervisorTimedOverrideTimed) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);
    
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
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}

TEST_F(ThrustControlSupervisorTest, StepSupervisorTimedOverrideUntimed) {
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position(0.0f, 1.5f, 1.8f, -99.0f, 0.0f, 0.0f);
    Position waypoints(-9.0f, 10.0f, 42.5f, 42.0f, 0.0f, 1.0f);
    
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
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);

    supervisor.step(FeedForward, position, waypoints);
    EXPECT_EQ(supervisor.get_control_mode(), FeedForward);
    EXPECT_EQ(supervisor.get_current_position(), position);
    EXPECT_EQ(supervisor.get_waypoint(), waypoints);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}



// Test overloaded push_to_pwm_queue function with untimed command, non-override
TEST_F(ThrustControlSupervisorTest, PushToQueueUntimedNonOverride) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array test_pwm = {1600, 1700, 1400, 1300, 1800, 1200, 1650, 1550};
    
    supervisor.push_to_pwm_queue(test_pwm, 0.0f, false, false);
    
    supervisor.step(FeedForward, position, waypoint);
     
     // Step many times to ensure the timed command is not overridden
    for (int i = 0; i < 100; i++) {
        supervisor.step(FeedForward, position, waypoint);
    }
    
    EXPECT_EQ(supervisor.get_current_pwm(), test_pwm);
}

// Test overloaded push_to_pwm_queue function with untimed command, override
TEST_F(ThrustControlSupervisorTest, PushToQueueUntimedOverride) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array initial_pwm = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    pwm_array override_pwm = {1800, 1200, 1900, 1100, 1700, 1300, 1600, 1400};
    
    supervisor.push_to_pwm_queue(initial_pwm, 0.0f, false, false);
    
    supervisor.push_to_pwm_queue(override_pwm, 0.0f, false, true);
    
    supervisor.step(FeedForward, position, waypoint);
    
    EXPECT_EQ(supervisor.get_current_pwm(), override_pwm);
}

// Test overloaded push_to_pwm_queue function with timed command, non-override
TEST_F(ThrustControlSupervisorTest, PushToQueueTimedNonOverride) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array test_pwm = {1650, 1350, 1750, 1250, 1850, 1150, 1550, 1450};
    float duration = 100.0f; // 100ms
    
    supervisor.push_to_pwm_queue(test_pwm, duration, true, false);
    
    supervisor.step(FeedForward, position, waypoint);
    
   
    EXPECT_EQ(supervisor.get_current_pwm(), test_pwm);
}

// Test overloaded push_to_pwm_queue function with timed command, override
TEST_F(ThrustControlSupervisorTest, PushToQueueTimedOverride) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array initial_pwm = {1600, 1400, 1600, 1400, 1600, 1400, 1600, 1400};
    pwm_array timed_override_pwm = {1900, 1100, 1800, 1200, 1700, 1300, 1900, 1100};
    float duration = 50.0f; // 50ms
    
    supervisor.push_to_pwm_queue(initial_pwm, 0.0f, false, false);
    
    supervisor.push_to_pwm_queue(timed_override_pwm, duration, true, true);
    
    supervisor.step(FeedForward, position, waypoint);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(supervisor.get_current_pwm().pwm_signals[i], timed_override_pwm.pwm_signals[i]);
    }
}

// Test multiple commands with overloaded function
TEST_F(ThrustControlSupervisorTest, PushToQueueMultipleCommands) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array pwm_1 = {1600, 1400, 1700, 1300, 1800, 1200, 1650, 1350};
    pwm_array pwm_2 = {1550, 1450, 1650, 1350, 1750, 1250, 1600, 1400};
    
    supervisor.push_to_pwm_queue(pwm_1, 0.0f, false, false);
    supervisor.push_to_pwm_queue(pwm_2, 0.0f, false, false);
    
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_1);
    
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
    
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_2);
}

// Test that duration parameter is properly handled for untimed commands
TEST_F(ThrustControlSupervisorTest, PushToQueueUntimedIgnoresDuration) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array test_pwm = {1700, 1300, 1800, 1200, 1600, 1400, 1750, 1250};
    
    supervisor.push_to_pwm_queue(test_pwm, 1000.0f, false, false);
    
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), test_pwm);
    
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), test_pwm);
}

// Test override clears command queue
TEST_F(ThrustControlSupervisorTest, PushToQueueOverrideClearsQueue) {    
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array queued_pwm = {1600, 1400, 1600, 1400, 1600, 1400, 1600, 1400};
    pwm_array override_pwm = {1800, 1200, 1800, 1200, 1800, 1200, 1800, 1200};
    pwm_array after_override_pwm = {1700, 1300, 1700, 1300, 1700, 1300, 1700, 1300};
    
    supervisor.push_to_pwm_queue(queued_pwm, 0.0f, false, false);
    supervisor.push_to_pwm_queue(queued_pwm, 0.0f, false, false);
    
    supervisor.push_to_pwm_queue(override_pwm, 0.0f, false, true);
    
    supervisor.push_to_pwm_queue(after_override_pwm, 0.0f, false, false);
    
    supervisor.step(FeedForward, position, waypoint);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(supervisor.get_current_pwm().pwm_signals[i], override_pwm.pwm_signals[i]);
    }
    
    supervisor.step(FeedForward, position, waypoint);
    for (int i = 0; i < 8; i++) {   
        EXPECT_EQ(supervisor.get_current_pwm().pwm_signals[i], after_override_pwm.pwm_signals[i]);
    }
}

// test that timed commands expire
TEST_F(ThrustControlSupervisorTest, TimedCommandsExpire) {   
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());

    Position position;
    Position waypoint;
    
    pwm_array pwm_0 = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    pwm_array pwm_1 = {1600, 1400, 1600, 1400, 1600, 1400, 1600, 1400};

    supervisor.push_to_pwm_queue(pwm_1, 500.0f, true, false);

    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_1);

    std::this_thread::sleep_for(std::chrono::milliseconds(490));
    supervisor.step(FeedForward, position, waypoint);
    EXPECT_EQ(supervisor.get_current_pwm(), pwm_1);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    supervisor.step(FeedForward, position, waypoint);

    EXPECT_EQ(supervisor.get_current_pwm(), pwm_0);

    
}

TEST_F(ThrustControlSupervisorTest, LimitCommand) {
    Position position;
    Position waypoint;
    thrust_control::ThrustControlSupervisor supervisor(
        logger, 
        std::move(interpreter),
        thrust_control::CommandQueue());
    pwm_array pwm = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800};
    pwm_array expected_pwm = {1400, 1400, 1400, 1400, 1500, 1600, 1600, 1600};
    std::unique_ptr<SupervisorCommand> command = std::make_unique<Untimed_Command>(pwm, false);
    supervisor.set_pwm_limit(1400, 1600);
    supervisor.push_to_pwm_queue(pwm, 0.0f, false, true);
    supervisor.step(FeedForward, position, waypoint);


    // ugly assert block fived better terminal output
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(supervisor.get_current_pwm().pwm_signals[i], expected_pwm.pwm_signals[i]);
    }
}
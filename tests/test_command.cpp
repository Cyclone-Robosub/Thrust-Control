#include <gtest/gtest.h>
#include <memory>
#include "supervisor_command.hpp"
#include <chrono>
#include "sets.hpp"

using namespace thrust_control;


// class CommandQueueTest : public ::testing::Test {
// protected:
//     CommandQueueTest(){} 

//     void SetUp() override 
//     {
//         CQ = new thrust_control::CommandQueue();
//     }

//     void TearDown() override 
//     {
//         delete CQ;
//         CQ = nullptr;
//     }

//     thrust_control::CommandQueue* CQ;

// };

TEST(CommandTest, CreateUntimedCommand) {
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(stop_set);

    ASSERT_EQ(untimed_command->getPwms(), stop_set);
    ASSERT_EQ(untimed_command->isFinished(), false);
    ASSERT_EQ(untimed_command->isOverride(), true);
    ASSERT_EQ(untimed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, CreateUntimedCommandCustomPWM) {
    pwm_array pwm = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(pwm);

    ASSERT_EQ(untimed_command->getPwms(), pwm);
    ASSERT_EQ(untimed_command->isFinished(), false);
    ASSERT_EQ(untimed_command->isOverride(), true);
    ASSERT_EQ(untimed_command->onExpirePwm(), pwm);
}

TEST(CommandTest, CreateUntimedNoOverride) {
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(stop_set, false);

    ASSERT_EQ(untimed_command->getPwms(), stop_set);
    ASSERT_EQ(untimed_command->isFinished(), false);
    ASSERT_EQ(untimed_command->isOverride(), false);
    ASSERT_EQ(untimed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, ExecuteUntimed) {
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(stop_set);

    untimed_command->start();

    ASSERT_EQ(untimed_command->getPwms(), stop_set);
    ASSERT_EQ(untimed_command->isFinished(), true);
    ASSERT_EQ(untimed_command->isOverride(), true);
    ASSERT_EQ(untimed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, ExecuteUntimedNoOverride) {
    std::unique_ptr<SupervisorCommand> untimed_command = std::make_unique<Untimed_Command>(stop_set, false);

    untimed_command->start();

    ASSERT_EQ(untimed_command->getPwms(), stop_set);
    ASSERT_EQ(untimed_command->isFinished(), true);
    ASSERT_EQ(untimed_command->isOverride(), false);
    ASSERT_EQ(untimed_command->onExpirePwm(), stop_set);
}




TEST(CommandTest, CreateTimedCommand) {
    auto duration = std::chrono::milliseconds(1000);
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(stop_set, duration);

    ASSERT_EQ(timed_command->getPwms(), stop_set);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), false);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, CreateTimedCommandCustomPWM) {
    auto duration = std::chrono::milliseconds(1000);
    pwm_array pwm = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(pwm, duration);

    ASSERT_EQ(timed_command->getPwms(), pwm);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), false);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, CreateTimedWithOverride) {
    auto duration = std::chrono::milliseconds(1000);
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(stop_set, duration, true);

    ASSERT_EQ(timed_command->getPwms(), stop_set);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), true);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);
}

TEST(CommandTest, ExecuteTimedOneSecond) {
    auto duration = std::chrono::milliseconds(1000);
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(stop_set, duration);

    ASSERT_EQ(timed_command->getPwms(), stop_set);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), false);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);

    auto startTime = std::chrono::steady_clock::now();
    timed_command->start();
    ASSERT_EQ(timed_command->isFinished(), false);
    while (!timed_command->isFinished()) {}
    auto endTime = std::chrono::steady_clock::now();

    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(1000) /
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
}

TEST(CommandTest, ExecuteTimedTwoSeconds) {
    auto duration = std::chrono::milliseconds(2000);
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(stop_set, duration);

    ASSERT_EQ(timed_command->getPwms(), stop_set);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), false);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);

    auto startTime = std::chrono::steady_clock::now();
    timed_command->start();
    ASSERT_EQ(timed_command->isFinished(), false);
    while (!timed_command->isFinished()) {}
    auto endTime = std::chrono::steady_clock::now();

    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(2000) /
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
}

TEST(CommandTest, ExecuteTimedWithOverride) {
    auto duration = std::chrono::milliseconds(1000);
    std::unique_ptr<SupervisorCommand> timed_command = std::make_unique<Timed_Command>(stop_set, duration, true);

    ASSERT_EQ(timed_command->getPwms(), stop_set);
    ASSERT_EQ(timed_command->isFinished(), false);
    ASSERT_EQ(timed_command->isOverride(), true);
    ASSERT_EQ(timed_command->onExpirePwm(), stop_set);

    auto startTime = std::chrono::steady_clock::now();
    timed_command->start();
    ASSERT_EQ(timed_command->isFinished(), false);
    while (!timed_command->isFinished()) {}
    auto endTime = std::chrono::steady_clock::now();

    ASSERT_NEAR((endTime - startTime) / std::chrono::milliseconds(1), std::chrono::milliseconds(1000) /
        std::chrono::milliseconds(1),
        std::chrono::milliseconds(10) / std::chrono::milliseconds(1));
}
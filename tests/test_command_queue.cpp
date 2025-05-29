#include <gtest/gtest.h>
#include <memory>
#include "command_interpreter_pointer.hpp"
#include "supervisor_command.hpp"
#include "command_queue.hpp"
#include "sets.hpp"

using namespace thrust_control;

class CommandQueueTest : public ::testing::Test {
protected:
    CommandQueueTest(){} 

    void SetUp() override 
    {
        CQ = thrust_control::CommandQueue();
    }

    thrust_control::CommandQueue CQ;

};


// Test 1:  Empty Queue Behaviour
TEST_F(CommandQueueTest, StopSetOnEnd) 
{
    std::unique_ptr<thrust_control::SupervisorCommand> empty_queue_response;

    
    std::unique_ptr<thrust_control::SupervisorCommand> test_command_ptr = 
        std::make_unique<Untimed_Command>(stop_set);

    empty_queue_response = std::move(CQ.get_command_from_queue(std::move(test_command_ptr)));

    EXPECT_EQ(empty_queue_response->getPwms(), stop_set);
}

TEST_F(CommandQueueTest, CorrectOrderOnPush) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    pwm_array pwm_3 = {1200, 1300, 1400, 1100, 1311, 1312, 1313, 1314};
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_1));
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_2));
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_3));

    std::unique_ptr<SupervisorCommand> new_command;

    new_command = CQ.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command->getPwms(), pwm_1);

    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), pwm_2);

    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), pwm_3);
    }

TEST_F(CommandQueueTest, CorrectPWMOnEmptiedUntimed) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_1));

    std::unique_ptr<SupervisorCommand> new_command;

    new_command = CQ.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command->getPwms(), pwm_1);

    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), pwm_1);
    
    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), pwm_1);
}

TEST_F(CommandQueueTest, CorrectPWMOnEmptiedTimed) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    auto duration = std::chrono::milliseconds(50);
    CQ.push_command(std::make_unique<Timed_Command>(pwm_1, duration));
    CQ.push_command(std::make_unique<Timed_Command>(pwm_2, duration));


    std::unique_ptr<SupervisorCommand> new_command;

    new_command = CQ.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command->getPwms(), pwm_1);

    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), pwm_2);
    
    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), stop_set);

    new_command = CQ.get_command_from_queue(std::move(new_command));
    EXPECT_EQ(new_command->getPwms(), stop_set);
}

TEST_F(CommandQueueTest, MakeNewCommand) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    pwm_array pwm_2 = {1500, 1900, 1240, 1100, 1300, 1500, 1900, 1500};
    auto duration = std::chrono::milliseconds(50);

    SupervisorCommand* command_1 = CQ.make_new_command(pwm_1, false, true).release();
    SupervisorCommand* command_2 = CQ.make_new_command(pwm_2, true, true, duration).release();

    Untimed_Command* untimed_pointer;
    Timed_Command* timed_pointer;

    untimed_pointer = dynamic_cast<Untimed_Command*>(command_1);
    timed_pointer = dynamic_cast<Timed_Command*>(command_2);

    EXPECT_NE(untimed_pointer, nullptr);
    EXPECT_NE(timed_pointer, nullptr);

    untimed_pointer = dynamic_cast<Untimed_Command*>(command_2);
    timed_pointer = dynamic_cast<Timed_Command*>(command_1);

    EXPECT_EQ(untimed_pointer, nullptr);
    EXPECT_EQ(timed_pointer, nullptr);
}

TEST_F(CommandQueueTest, AssignmentOperator) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_1));
    auto CQ_other = thrust_control::CommandQueue();

    CQ_other = CQ;

    auto new_command_other = CQ_other.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command_other->getPwms(), pwm_1);

    auto new_command = CQ.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command->getPwms(), pwm_1);
}

TEST_F(CommandQueueTest, CopyConstructor) {
    pwm_array pwm_1 = {1900, 1500, 1100, 1240, 1240, 1900, 1500, 1500};
    CQ.push_command(std::make_unique<Untimed_Command>(pwm_1));
    auto CQ_other = thrust_control::CommandQueue(CQ);

    auto new_command_other = CQ_other.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command_other->getPwms(), pwm_1);

    auto new_command = CQ.get_command_from_queue(std::make_unique<Untimed_Command>(stop_set));
    EXPECT_EQ(new_command->getPwms(), pwm_1);
}

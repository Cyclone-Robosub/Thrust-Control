#include <gtest/gtest.h>
#include <memory>
#include "command_interpreter_pointer.hpp"
#include "supervisor_command.hpp"
#include "command_queue.hpp"
#include "sets.hpp"


class CommandQueueTest : public ::testing::Test {
protected:
    CommandQueueTest(){} 

    void SetUp() override 
    {
        CQ = new thrust_control::CommandQueue();
    }

    void TearDown() override 
    {
        delete CQ;
        CQ = nullptr;
    }

    thrust_control::CommandQueue* CQ;

};


// Test 1:  Empty Queue Behaviour
TEST_F(CommandQueueTest, stop_set_on_end) 
{
    using namespace thrust_control;
    std::unique_ptr<thrust_control::SupervisorCommand> empty_queue_response;

    
    std::unique_ptr<thrust_control::SupervisorCommand> test_command_ptr = 
        std::make_unique<Untimed_Command>(stop_set);

    empty_queue_response = std::move(CQ->get_command_from_queue(std::move(test_command_ptr)));

    EXPECT_EQ(empty_queue_response->getPwms(), stop_set);
}
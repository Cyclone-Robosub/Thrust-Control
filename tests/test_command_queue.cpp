#include <gtest/gtest.h>
#include <memory>
#include "../src/command_interpreter_pointer.hpp"
#include "../src/supervisor_command.hpp"
#include "../src/command_queue.hpp"


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
TEST_F(CommandQueueTest, empty_queue_behaviour) 
{
    std::unique_ptr<thrust_control::SupervisorCommand> empty_queue_response;
    empty_queue_response = std::move(CQ->get_command_from_queue());

    pwm_array stop_set = {{1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}};

    for (int i = 0; i < 8; i++) 
    {
        EXPECT_EQ(empty_queue_response->getPwms().pwm_signals[i], stop_set.pwm_signals[i]);
    }
}



int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 

#include <gtest/gtest.h>
#include <memory>
#include "../src/command_interpreter_pointer.hpp"
#include "../src/supervisor_command.hpp"
#include "../src/command_queue.hpp"


class CommandQueueTest : public ::testing::Test {
protected:
    CommandQueueTest(){} 

    void SetUp() override {
        CQ = new thrust_control::CommandQueue();
    }

    void TearDown() override {
        delete CQ;
        CQ = nullptr;
    }

    thrust_control::CommandQueue* CQ;

};


// Test 1: Initialization
TEST_F(CommandQueueTest, CanBeInitialized) {
    std::unique_ptr<thrust_control::SupervisorCommand> empty_queue_response;
    empty_queue_response = CQ->get_command_from_queue();
    
}



int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 

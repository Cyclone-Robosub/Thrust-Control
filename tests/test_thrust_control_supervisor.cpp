#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter/src/Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"

class ThrustControlSupervisorTest : public ::testing::Test {
protected:

    ThrustControlSupervisorTest() : logger(rclcpp::get_logger("test_logger")) {}
    

    rclcpp::Node::SharedPtr test_node_sub_;
    rclcpp::Node::SharedPtr test_node_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

    void SetUp() override 
    {
        if (!rclcpp::ok()) {  rclcpp::init(0, nullptr);}
        

        // configure test node
        test_node_sub_ = std::make_shared<rclcpp::Node>("test_subscriber_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_node_sub_);
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
                std::move(interpreter));
    });
}

TEST_F(ThrustControlSupervisorTest, StepSupervisor) {

    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);

    thrust_control::ThrustControlSupervisor supervisor(
            logger, 
            std::move(interpreter));

    
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 

#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "thrust_control_node.hpp"


class ThrustControlNodeTest : public ::testing::Test {

protected:
    
    ThrustControlNodeTest() : logger(rclcpp::get_logger("test_logger")) {}
 
    rclcpp::Node::SharedPtr test_node_sub_;
    rclcpp::Node::SharedPtr test_node_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

    void SetUp() override
    { 
        if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
       
        test_node_sub_ = std::make_shared<rclcpp::Node>("test_subscriber_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_node_sub_);

    }

    void TearDown() override 
    {
        if (rclcpp::ok()) { rclcpp::shutdown(); }
    }

    rclcpp::Logger logger;
    std::ofstream nullOut = std::ofstream("/dev/null");
};

TEST_F(ThrustControlNodeTest, Con){

    auto interpreter = make_command_interpreter_ptr(
            nullOut, 
            nullOut, 
            std::cerr);
    auto node =  std::make_shared<thrust_control::ThrustControlNode>(
                std::move(interpreter));

    ASSERT_NO_THROW({rclcpp::spin_some(node); });
    node.reset();
}


TEST_F(ThrustControlNodeTest, ConstructorWithIntrepreter){
    auto interpreter = make_command_interpreter_ptr(
            nullOut, 
            nullOut, 
            std::cerr);
    auto node =         
            std::make_shared<thrust_control::ThrustControlNode>(
                    std::move(interpreter));
    ASSERT_NO_THROW({rclcpp::spin(node);});
    node.reset();

}
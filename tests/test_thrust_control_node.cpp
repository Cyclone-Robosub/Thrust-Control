#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "../src/thrust_control_supervisor.hpp"
#include "../include/Command_Interpreter/src/Command_Interpreter.hpp"
#include "../src/command_interpreter_pointer.hpp"
#include "../src/thrust_control_node.hpp"


class ThrustControlNodeTest : public ::testing::Test {

protected:
    ThrustControlNodeTest()
        : logger(rclcpp::get_logger("test_logger")) {
    }

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    rclcpp::Logger logger;
};

TEST_F(ThrustControlNodeTest, Con){

    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);
    auto node =  std::make_shared<thrust_control::ThrustControlNode>(
                std::move(interpreter));

    ASSERT_NO_THROW({rclcpp::spin_some(node); });
    node.reset();
}


TEST_F(ThrustControlNodeTest, ConsturctorWithIntrepreter){
    auto interpreter = make_command_interpreter_ptr(
            std::cout, 
            std::cout, 
            std::cout);
    auto node =         
            std::make_shared<thrust_control::ThrustControlNode>(
                    std::move(interpreter));
    ASSERT_NO_THROW({rclcpp::spin(node);});
    node.reset();

}




int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
          


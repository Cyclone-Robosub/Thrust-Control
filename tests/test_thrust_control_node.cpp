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
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_publisher_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    
    std::unique_ptr<Command_Interpreter_RPi5> interpreter_;
    std::shared_ptr<thrust_control::ThrustControlNode> thrust_node_;

    void SetUp() override
    { 
        if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
       
        test_node_sub_ = std::make_shared<rclcpp::Node>("test_subscriber_node");
        test_node_pub_ = std::make_shared<rclcpp::Node>("test_publisher_node");
        
        pwm_publisher_ = test_node_pub_->create_publisher<std_msgs::msg::Int32MultiArray>(
            "array_Cltool_topic", 10);
            
        interpreter_ = make_command_interpreter_ptr(std::cout, std::cout, std::cout);
        thrust_node_ = std::make_shared<thrust_control::ThrustControlNode>(std::move(interpreter_));
            
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(test_node_sub_);
        executor_->add_node(test_node_pub_);
        executor_->add_node(thrust_node_);
    }

    void TearDown() override 
    {
        thrust_node_.reset();
        if (rclcpp::ok()) { rclcpp::shutdown(); }
    }

    rclcpp::Logger logger;
};


TEST_F(ThrustControlNodeTest, PWMSubscriptionCallback) {
    auto pwm_msg = std_msgs::msg::Int32MultiArray();
    pwm_msg.data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};
    
    pwm_publisher_->publish(pwm_msg);
    
    ASSERT_NO_THROW({
        executor_->spin_some();
    });
}


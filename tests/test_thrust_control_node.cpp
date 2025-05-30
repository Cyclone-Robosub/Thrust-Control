#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp" 
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "thrust_control_node.hpp"
#include "pwm_message_utils.hpp"

class ThrustControlNodeTest : public ::testing::Test {

protected:
    
    ThrustControlNodeTest() : logger(rclcpp::get_logger("test_logger")) {}
 
    rclcpp::Node::SharedPtr test_node_sub_;
    rclcpp::Node::SharedPtr test_node_pub_;
    
    rclcpp::Publisher<crs_ros2_interfaces::msg::PwmCmd>::SharedPtr pwm_cmd_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_publisher_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    
    std::unique_ptr<Command_Interpreter_RPi5> interpreter_;
    std::shared_ptr<thrust_control::ThrustControlNode> thrust_node_;
    
    pwm_array received_pwm_data_;
    bool pwm_received_ = false;

    void SetUp() override
    { 
        if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
       
        test_node_sub_ = std::make_shared<rclcpp::Node>("test_subscriber_node");
        test_node_pub_ = std::make_shared<rclcpp::Node>("test_publisher_node");
        
        pwm_cmd_publisher_ = test_node_pub_->create_publisher<crs_ros2_interfaces::msg::PwmCmd>(
            "pwm_cmd_topic", 10);
            
        pwm_subscriber_ = test_node_sub_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "sent_pwm_topic", 10,
            [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 8; i++) {
                    received_pwm_data_.pwm_signals[i] = msg->data[i];
                }
                pwm_received_ = true;
            });
            
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
    std::ofstream nullOut = std::ofstream("/dev/null");
};



TEST_F(ThrustControlNodeTest, PWMSubscriptionCallback) {
    auto pwm_msg = pwm_utils::create_pwm_msg(std::array<int, 8>{1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700});
    
    pwm_cmd_publisher_->publish(pwm_msg);
    
    ASSERT_NO_THROW({
        executor_->spin_some();
    });
}

TEST_F(ThrustControlNodeTest, PWMCallbackStoresDataCorrectly) {
    // Send specific PWM values using the custom message
    auto pwm_msg = crs_ros2_interfaces::msg::PwmCmd();
    pwm_msg.pwm_flt = 1100;
    pwm_msg.pwm_frt = 1200;
    pwm_msg.pwm_rlt = 1300;
    pwm_msg.pwm_rrt = 1400;
    pwm_msg.pwm_flb = 1500;
    pwm_msg.pwm_frb = 1600;
    pwm_msg.pwm_rlb = 1700;
    pwm_msg.pwm_rrb = 1800;
    pwm_msg.is_timed = false;
    pwm_msg.duration = 0;
    pwm_msg.is_overriding = false;
    
    // Publish the PWM data
    pwm_cmd_publisher_->publish(pwm_msg);
    
    // Process the callback
    executor_->spin_some();

    // Get the stored PWM data directly from the node using the getter
    pwm_array stored_pwm = thrust_node_->get_pwm();
    
    // Check that the stored data is correct
    int expected_values[] = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800};
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(stored_pwm.pwm_signals[i], expected_values[i]) << "PWM signal " << i << " is incorrect";
    }
}

TEST_F(ThrustControlNodeTest, PWMCallbackStoresAdditionalDataCorrectly) {
    // Test that the callback also stores duration, manual_override, and is_timed_command correctly
    auto pwm_msg = crs_ros2_interfaces::msg::PwmCmd();
    pwm_msg.pwm_flt = 1200;
    pwm_msg.pwm_frt = 1300;
    pwm_msg.pwm_rlt = 1400;
    pwm_msg.pwm_rrt = 1500;
    pwm_msg.pwm_flb = 1600;
    pwm_msg.pwm_frb = 1700;
    pwm_msg.pwm_rlb = 1800;
    pwm_msg.pwm_rrb = 1900;
    pwm_msg.is_timed = true;
    pwm_msg.duration = 5000;  // 5 seconds
    pwm_msg.is_overriding = true;
    
    // Publish the PWM data
    pwm_cmd_publisher_->publish(pwm_msg);
    
    // Process the callback
    executor_->spin_some();
    
    // Check that all data is stored correctly
    EXPECT_EQ(thrust_node_->get_duration(), 5000);
    EXPECT_EQ(thrust_node_->get_manual_override(), true);
    EXPECT_EQ(thrust_node_->get_is_timed_command(), true);
}

TEST_F(ThrustControlNodeTest, ManualPWMPublishedOnSentTopic) {
    auto pwm_msg = crs_ros2_interfaces::msg::PwmCmd();
    pwm_msg.pwm_flt = 1400;
    pwm_msg.pwm_frt = 1450;
    pwm_msg.pwm_rlt = 1500;
    pwm_msg.pwm_rrt = 1550;
    pwm_msg.pwm_flb = 1600;
    pwm_msg.pwm_frb = 1350;
    pwm_msg.pwm_rlb = 1650;
    pwm_msg.pwm_rrb = 1700;
    pwm_msg.is_timed = false;
    pwm_msg.duration = 0;
    pwm_msg.is_overriding = false;
    
    // Reset the flag
    pwm_received_ = false;
    
    // Publish input PWM
    pwm_cmd_publisher_->publish(pwm_msg);
    
    // Spin until we receive data or timeout (wait longer than 1 second for timer)
    auto start = std::chrono::steady_clock::now();
    while (!pwm_received_ && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2)) {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Check that PWM was published
    EXPECT_TRUE(pwm_received_) << "No PWM data received on sent_pwm_topic";
    
    if (pwm_received_) {
        // Check that we got 8 PWM values
        // Note: The actual values will be processed by the supervisor, so they might be different
        for (int i = 0; i < 8; i++) {
            EXPECT_GE(received_pwm_data_.pwm_signals[i], 1100) << "PWM signal " << i << " too low";
            EXPECT_LE(received_pwm_data_.pwm_signals[i], 1900) << "PWM signal " << i << " too high";
        }
    }
}

TEST_F(ThrustControlNodeTest, TimerCallbackExecutes) {
    // Reset the flag
    pwm_received_ = false;
    
    auto start = std::chrono::steady_clock::now();
    while (!pwm_received_ && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(1)) {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_TRUE(pwm_received_) << "Timer callback did not execute - no PWM data published";
    
    if (pwm_received_) {
        for (int i = 0; i < 8; i++) {
            EXPECT_GE(received_pwm_data_.pwm_signals[i], 1000);
            EXPECT_LE(received_pwm_data_.pwm_signals[i], 2000);
        }
    }
}



TEST_F(ThrustControlNodeTest, Constructor) {
    auto interpreter = make_command_interpreter_ptr(
            nullOut, 
            nullOut, 
            std::cerr);
    auto node =         
            std::make_shared<thrust_control::ThrustControlNode>(
                    std::move(interpreter));
    ASSERT_NO_THROW({rclcpp::spin_some(node);});
    node.reset();
}

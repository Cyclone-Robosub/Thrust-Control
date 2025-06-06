#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <ostream>
#include "rclcpp/rclcpp.hpp" 
#include "thrust_control_supervisor.hpp"
#include "Command_Interpreter.hpp"
#include "command_interpreter_pointer.hpp"
#include "thrust_control_node.hpp"
#include "pwm_message_utils.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class ThrustControlNodeTest : public ::testing::Test {

protected:
    
    ThrustControlNodeTest() : logger(rclcpp::get_logger("test_logger")) {}
 
    rclcpp::Node::SharedPtr test_node_sub_;
    rclcpp::Node::SharedPtr test_node_pub_;
    
    rclcpp::Publisher<crs_ros2_interfaces::msg::PwmCmd>::SharedPtr pwm_cmd_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_mode_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_publisher_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    
    std::unique_ptr<Command_Interpreter_RPi5> interpreter_;
    std::shared_ptr<thrust_control::ThrustControlNode> thrust_node_;
    
    pwm_array received_pwm_data_;
    bool pwm_received_ = false;
    std::ofstream nullOut = std::ofstream("/dev/null");

    void SetUp() override
    { 
        if (!rclcpp::ok()) { rclcpp::init(0, nullptr); }
        //testing::internal::CaptureStdout();
       
        test_node_sub_ = std::make_shared<rclcpp::Node>("test_subscriber_node");
        test_node_pub_ = std::make_shared<rclcpp::Node>("test_publisher_node");
        
        pwm_cmd_publisher_ = test_node_pub_->create_publisher<crs_ros2_interfaces::msg::PwmCmd>(
            "pwm_cmd_topic", 10);

        control_mode_publisher_ = test_node_pub_->create_publisher<std_msgs::msg::String>(
            "control_mode_topic", 10);

        position_publisher_ = test_node_pub_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "position_topic", 10);

        waypoint_publisher_ = test_node_pub_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "waypoint_topic", 10);
            
        pwm_subscriber_ = test_node_sub_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "sent_pwm_topic", 10, [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) 
            {
                for (int i = 0; i < 8; i++) {
                    received_pwm_data_.pwm_signals[i] = msg->data[i];
                }
                pwm_received_ = true;
            });
        
        // logFile.open("../logFile.txt");
        interpreter_ = make_command_interpreter_ptr(nullOut, nullOut, std::cerr);
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
        // logFile.close();
    }

    rclcpp::Logger logger;
};



TEST_F(ThrustControlNodeTest, PWMSubscriptionCallback) {

    pwm_array pwm_data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};

    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, false, 0, false);
    
    pwm_cmd_publisher_->publish(pwm_msg);
    
    ASSERT_NO_THROW({
        executor_->spin_some();
    });
}

TEST_F(ThrustControlNodeTest, PWMCallbackStoresDataCorrectly) {
    // Send specific PWM values using the custom message
    pwm_array pwm_data = {1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800};
    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, false, 0, false);
    
    pwm_cmd_publisher_->publish(pwm_msg);
    executor_->spin_some();

    pwm_array stored_pwm = thrust_node_->get_user_pwm();
    
    EXPECT_EQ(stored_pwm, pwm_data);
    EXPECT_EQ(thrust_node_->get_duration(), 0);
    EXPECT_EQ(thrust_node_->get_manual_override(), false);
    EXPECT_EQ(thrust_node_->get_is_timed_command(), false);
}

TEST_F(ThrustControlNodeTest, ControlModeCallbackStoresDataCorrectly) {

    auto control_mode_msg = std_msgs::msg::String();
    
    control_mode_msg.data = "FeedForward";
    control_mode_publisher_->publish(control_mode_msg);
    executor_->spin_some();
    EXPECT_EQ(thrust_node_->get_control_mode(), thrust_control::ControlMode::FeedForward);

    control_mode_msg.data = "PID";
    control_mode_publisher_->publish(control_mode_msg);
    executor_->spin_some();
    EXPECT_EQ(thrust_node_->get_control_mode(), thrust_control::ControlMode::PID);

    control_mode_msg.data = "STOP";
    control_mode_publisher_->publish(control_mode_msg);
    executor_->spin_some();
    EXPECT_EQ(thrust_node_->get_control_mode(), thrust_control::ControlMode::STOP);
}
TEST_F(ThrustControlNodeTest, ManualPWMRetreivedCorrectly)
{
    pwm_array pwm_data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};
    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, false, 0, true);
    auto control_mode_msg = std_msgs::msg::String();
    control_mode_msg.data = "FeedForward";
    pwm_received_ = false;
    
    control_mode_publisher_->publish(control_mode_msg);
    executor_->spin_some();

    pwm_cmd_publisher_->publish(pwm_msg);
    
    auto start = std::chrono::steady_clock::now();
    while (!pwm_received_ || std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) 
    {
        executor_->spin_some();  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    EXPECT_EQ(thrust_node_->get_thruster_pwm(), pwm_data);
}

TEST_F(ThrustControlNodeTest, ManualPWMPublishedOnSentTopic) 
{
    pwm_array pwm_data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};
    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, false, 0, true);
    auto control_mode_msg = std_msgs::msg::String();
    control_mode_msg.data = "FeedForward";
    pwm_received_ = false;
    
    pwm_cmd_publisher_->publish(pwm_msg);
    executor_->spin_some();
    control_mode_publisher_->publish(control_mode_msg);
    
    auto start = std::chrono::steady_clock::now();
    while (!pwm_received_ && std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) 
    {
        executor_->spin_some();  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
    EXPECT_TRUE(pwm_received_) << "No PWM data received on sent_pwm_topic";
}

TEST_F(ThrustControlNodeTest, ManualPWMPublishedOnSentTopicAndReceivedAccuaratly) 
{
    pwm_array pwm_data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};
    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, false, 0, true);
    auto control_mode_msg = std_msgs::msg::String();
    control_mode_msg.data = "FeedForward";
    pwm_received_ = false;
    
    control_mode_publisher_->publish(control_mode_msg);
    executor_->spin_some();
    pwm_cmd_publisher_->publish(pwm_msg);
    
    
    auto start = std::chrono::steady_clock::now();
    while (!pwm_received_ && std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) 
    {
        executor_->spin_some();  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    
    EXPECT_TRUE(pwm_received_) << "No PWM data received on sent_pwm_topic";
    
    if (pwm_received_) {
        EXPECT_EQ(received_pwm_data_, pwm_data);
        // Check that we got 8 PWM values
        // Note: The actual values will be processed by the supervisor, so they might be different
        for (int i = 0; i < 8; i++) {
            EXPECT_EQ(received_pwm_data_.pwm_signals[i], pwm_data.pwm_signals[i])
            << "PWM signal " << i << " is incorrect";
        }
    }
}
TEST_F(ThrustControlNodeTest, TimedCommandExpiresCorrectly)
{
    pwm_array pwm_data = {1400, 1450, 1500, 1550, 1600, 1350, 1650, 1700};
    pwm_array stop_pwm = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    auto pwm_msg = pwm_utils::create_pwm_msg(pwm_data, true, 1, true);
    pwm_cmd_publisher_->publish(pwm_msg);

    auto control_mode_msg = std_msgs::msg::String();
    control_mode_msg.data = "FeedForward";
    control_mode_publisher_->publish(control_mode_msg);

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)){
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(thrust_node_->get_thruster_pwm(), pwm_data) << "Initial pwm data is not correct";
    
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1100)){
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(thrust_node_->get_thruster_pwm(), stop_pwm) << "Stop pwm data is not correct";
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
            EXPECT_GE(received_pwm_data_.pwm_signals[i], 1200);
            EXPECT_LE(received_pwm_data_.pwm_signals[i], 1900);
        }
    }
}

TEST_F(ThrustControlNodeTest, PositionCallbackStoresDataCorrectly)
{
    std_msgs::msg::Float32MultiArray position_msg;
    Position position = {1, 2, 3, 4, 5, 6};
    position_msg.data = {
        position.get_x(), 
        position.get_y(), 
        position.get_z(), 
        position.get_roll(), 
        position.get_pitch(), 
        position.get_yaw()};
    position_publisher_->publish(position_msg);
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) 
    {
        executor_->spin_some();  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(thrust_node_->get_position(), position);
}
TEST_F(ThrustControlNodeTest, WaypointCallbackStoresDataCorrectly)
{ 
    std_msgs::msg::Float32MultiArray waypoint_msg;
    Position waypoint = {1, 2, 3, 4, 5, 6};
    waypoint_msg.data = {
        waypoint.get_x(), 
        waypoint.get_y(), 
        waypoint.get_z(), 
        waypoint.get_roll(), 
        waypoint.get_pitch(), 
        waypoint.get_yaw()};
    waypoint_publisher_->publish(waypoint_msg);
    
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) 
    {
        executor_->spin_some();  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(thrust_node_->get_waypoint(), waypoint);
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

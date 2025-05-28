#include "thrust_control_node.hpp"

namespace thrust_control
{

ThrustControlNode::ThrustControlNode
(std::unique_ptr<Command_Interpreter_RPi5> interpreter) 
    : Node("thrust_control_node"), 
    supervisor_(this->get_logger(), std::move(interpreter), CommandQueue())
{
    _manual_pwm_subscription =  
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
                manual_pwm_topic_, 
                10, std::bind(&ThrustControlNode::pwm_topic_callback, 
                    this, 
                    std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            sent_pwm_topic_, 
            10);
        
    timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ThrustControlNode::timer_callback, this));

}

void ThrustControlNode::pwm_topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received PWM array with %zu values", msg->data.size());
  
  // Print the values in the array
  std::stringstream ss;
  ss << "PWM values: ";
  for (size_t i = 0; i < msg->data.size(); ++i) {
    pwm_.pwm_signals[i] = msg->data[i];  
    ss << msg->data[i];
    if (i < msg->data.size() - 1) {
      ss << ", ";
    }
  }
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

void ThrustControlNode::timer_callback()
{
    
    std::stringstream ss;
    
    // this data should get replaced with data from callbacks
    // only exists as is here for testing putposes
    ControlMode test_mode = PID;
    std::array<float, 6> test_pos = {0,0,0,0,0,0};
    std::array<float, 6> waypoint = {0,0,0,0,0,0};
    
    supervisor_.step(
            test_mode,
            test_pos,
            waypoint);

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    std::lock_guard<std::mutex> lock(message_mutex_);
    if (last_message_ != nullptr) 
    {
        std::cout << "Nada\n";
        RCLCPP_INFO(
                this->get_logger(), 
                "Latest message on topic: '%s'", 
                last_message_->data.c_str());
    } 
    else 
    {
        RCLCPP_INFO(this->get_logger(), "No messages received yet on the topic");
    }
}

}// namespace thrust_control 

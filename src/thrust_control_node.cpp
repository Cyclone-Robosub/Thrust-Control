#include "thrust_control_node.hpp"

namespace thrust_control
{

ThrustControlNode::ThrustControlNode
(std::unique_ptr<Command_Interpreter_RPi5> interpreter) 
    : Node("thrust_control_node"), 
    supervisor_(this->get_logger(), std::move(interpreter), CommandQueue())
{

    _manual_pwm_subscription =  
        this->create_subscription<crs_ros2_interfaces::msg::PwmCmd>(
                manual_pwm_topic_, 
                10, std::bind(&ThrustControlNode::pwm_topic_callback, 
                    this, 
                    std::placeholders::_1));

    _pwm_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            sent_pwm_topic_, 
            10);
        
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&ThrustControlNode::timer_callback, this));

}
void ThrustControlNode::pwm_topic_callback(const crs_ros2_interfaces::msg::PwmCmd::SharedPtr msg)
{
    // consolidate pwm data into a single array
    pwm_array pwm_data;
    pwm_data.pwm_signals[0] = msg->pwm_flt;
    pwm_data.pwm_signals[1] = msg->pwm_frt;
    pwm_data.pwm_signals[2] = msg->pwm_rlt;
    pwm_data.pwm_signals[3] = msg->pwm_rrt;
    pwm_data.pwm_signals[4] = msg->pwm_flb;
    pwm_data.pwm_signals[5] = msg->pwm_frb;
    pwm_data.pwm_signals[6] = msg->pwm_rlb;
    pwm_data.pwm_signals[7] = msg->pwm_rrb;
    pwm_ = pwm_data;
    duration_ = msg->duration;
    manual_override_ = msg->is_overriding;
    is_timed_command_ = msg->is_timed;
}

void ThrustControlNode::timer_callback()
{
    
    std::stringstream ss;
    
    // this data should get replaced with data from callbacks
    // only exists as is here for testing putposes
    ControlMode test_mode = PID;
    Position test_pos = {0,0,0,0,0,0};
    Position waypoint = {0,0,0,0,0,0};
    
    supervisor_.step(
            test_mode,
            test_pos,
            waypoint);
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    send_pwm();

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

void ThrustControlNode::send_pwm()
{   pwm_array current_pwm = supervisor_.get_current_pwm();

    auto pwm_msg = std_msgs::msg::Int32MultiArray();
    pwm_msg.data.resize(8);
    for (int i = 0; i < 8; i++) {
        pwm_msg.data[i] = current_pwm.pwm_signals[i];
    }

    _pwm_publisher->publish(pwm_msg);
}
}// namespace thrust_control 

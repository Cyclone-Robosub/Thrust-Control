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

    _pwm_limit_subscription =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
                pwm_limit_topic_,
                10, std::bind(&ThrustControlNode::pwm_limit_callback,
                    this,
                    std::placeholders::_1));

    _control_mode_subscription =
        this->create_subscription<std_msgs::msg::String>(
                control_mode_topic_,
                10, std::bind(&ThrustControlNode::control_mode_callback,
                    this,
                    std::placeholders::_1));

    _position_subscription =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
                position_topic_,
                10, std::bind(&ThrustControlNode::position_callback,
                    this,
                    std::placeholders::_1));
    _voltage_subscription = 
        this->create_subscription<std_msgs::msg::Float64>(
                "voltageReadingTopic", rclcpp::QoS(5),
                std::bind(&ThrustControlNode::voltage_callback, this, std::placeholders::_1)
        );

    _waypoint_subscription =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
                waypoint_topic_,
                10, std::bind(&ThrustControlNode::waypoint_callback,
                this, std::placeholders::_1));
    _pwm_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            sent_pwm_topic_, 
            10);
        
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
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
    user_pwm_ = pwm_data;
    duration_ = msg->duration;
    manual_override_ = msg->is_overriding;
    is_timed_command_ = msg->is_timed;

    if (is_timed_command_)  
    {
        supervisor_.push_to_pwm_queue(std::make_unique<Timed_Command>(pwm_data, std::chrono::milliseconds(static_cast<int>(duration_ * 1000)), manual_override_));
    }
    else
    {
        supervisor_.push_to_pwm_queue(std::make_unique<Untimed_Command>(pwm_data, manual_override_));
    }
}

void ThrustControlNode::pwm_limit_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 2)
    {
        std::cout << "PWM limit is not the correct size" << std::endl;
        return;
    }

    int pwm_limit[2] = {msg->data[0], msg->data[1]};
    pwm_limit_[0] = std::min(pwm_limit[0], pwm_limit_[1]);
    pwm_limit_[1] = std::max(pwm_limit[1], pwm_limit_[0]);
    supervisor_.set_pwm_limit(pwm_limit_[0], pwm_limit_[1]);
}

void ThrustControlNode::control_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // Convert string to ControlMode enum
    std::string mode_str = msg->data;
    if (mode_str == "FeedForward") {
        control_mode_ = ControlMode::FeedForward;
    } else if (mode_str == "PID") {
        control_mode_ = ControlMode::PID;
    } else if (mode_str == "STOP") {
        control_mode_ = ControlMode::STOP;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown control mode: %s. Defaulting to FeedForward.", mode_str.c_str());
        control_mode_ = ControlMode::FeedForward;
    }
    
    RCLCPP_INFO(this->get_logger(), "Control mode set to: %s", mode_str.c_str());
}   
void ThrustControlNode::voltage_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if(msg->data <= 14.2){
        isLowVoltage = true;
        supervisor_.isLowVoltageReading = true;
    }
}
void ThrustControlNode::position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    position_ = Position(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
}

void ThrustControlNode::waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    waypoint_ = Position(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
}

void ThrustControlNode::timer_callback()
{
    ControlMode test_mode = control_mode_;
    Position test_pos = position_;
    Position waypoint = waypoint_;
    
    supervisor_.step(
            test_mode,
            test_pos,
            waypoint);
    
    thruster_pwm_ = supervisor_.get_current_pwm();
    send_pwm();
}

void ThrustControlNode::send_pwm()
{   

    auto pwm_msg = std_msgs::msg::Int32MultiArray();
    pwm_msg.data.resize(8);
    for (int i = 0; i < 8; i++) {
        pwm_msg.data[i] = thruster_pwm_.pwm_signals[i];
    }

    _pwm_publisher->publish(pwm_msg);
}
}// namespace thrust_control 

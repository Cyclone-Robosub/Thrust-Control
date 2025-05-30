#include "pwm_message_utils.hpp"

namespace pwm_utils {

crs_ros2_interfaces::msg::PwmCmd create_pwm_msg(
    const std::array<int, 8>& pwm_values,
    bool is_timed,
    int64_t duration,
    bool is_overriding
) {
    auto msg = crs_ros2_interfaces::msg::PwmCmd();
    
    // Set PWM values in the order: flt, frt, rlt, rrt, flb, frb, rlb, rrb
    msg.pwm_flt = pwm_values[0];
    msg.pwm_frt = pwm_values[1];
    msg.pwm_rlt = pwm_values[2];
    msg.pwm_rrt = pwm_values[3];
    msg.pwm_flb = pwm_values[4];
    msg.pwm_frb = pwm_values[5];
    msg.pwm_rlb = pwm_values[6];
    msg.pwm_rrb = pwm_values[7];
    
    // Set metadata
    msg.is_timed = is_timed;
    msg.duration = duration;
    msg.is_overriding = is_overriding;
    
    return msg;
}

crs_ros2_interfaces::msg::PwmCmd create_pwm_msg(
    const pwm_array& pwm_data,
    bool is_timed,
    int64_t duration,
    bool is_overriding
) {
    std::array<int, 8> pwm_values;
    for (int i = 0; i < 8; i++) {
        pwm_values[i] = pwm_data.pwm_signals[i];
    }
    
    return create_pwm_msg(pwm_values, is_timed, duration, is_overriding);
}

crs_ros2_interfaces::msg::PwmCmd create_uniform_pwm_msg(
    int pwm_value,
    bool is_timed,
    int64_t duration,
    bool is_overriding
) {
    std::array<int, 8> pwm_values;
    pwm_values.fill(pwm_value);
    
    return create_pwm_msg(pwm_values, is_timed, duration, is_overriding);
}

crs_ros2_interfaces::msg::PwmCmd create_stop_pwm_msg() {
    return create_uniform_pwm_msg(1500);  // 1500 is the stop value
}

} // namespace pwm_utils
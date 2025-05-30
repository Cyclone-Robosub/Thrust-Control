#ifndef PWM_MESSAGE_UTILS_HPP
#define PWM_MESSAGE_UTILS_HPP

#include "crs_ros2_interfaces/msg/pwm_cmd.hpp"
#include "Command.hpp"
#include <array>

namespace pwm_utils {

/**
 * @brief Create a PwmCmd message from an array of 8 PWM values
 * @param pwm_values Array of 8 PWM values [flt, frt, rlt, rrt, flb, frb, rlb, rrb]
 * @param is_timed Whether this is a timed command (default: false)
 * @param duration Duration in milliseconds (default: 0)
 * @param is_overriding Whether this command overrides existing ones (default: false)
 * @return PwmCmd message ready to publish
 */
crs_ros2_interfaces::msg::PwmCmd create_pwm_msg(
    const std::array<int, 8>& pwm_values,
    bool is_timed = false,
    int64_t duration = 0,
    bool is_overriding = false
);

/**
 * @brief Create a PwmCmd message from a pwm_array struct
 * @param pwm_array PWM array struct
 * @param is_timed Whether this is a timed command (default: false)
 * @param duration Duration in milliseconds (default: 0)
 * @param is_overriding Whether this command overrides existing ones (default: false)
 * @return PwmCmd message ready to publish
 */
crs_ros2_interfaces::msg::PwmCmd create_pwm_msg(
    const pwm_array& pwm_data,
    bool is_timed = false,
    int64_t duration = 0,
    bool is_overriding = false
);

/**
 * @brief Create a PwmCmd message with all thrusters set to the same value
 * @param pwm_value PWM value for all thrusters
 * @param is_timed Whether this is a timed command (default: false)
 * @param duration Duration in milliseconds (default: 0)
 * @param is_overriding Whether this command overrides existing ones (default: false)
 * @return PwmCmd message ready to publish
 */
crs_ros2_interfaces::msg::PwmCmd create_uniform_pwm_msg(
    int pwm_value,
    bool is_timed = false,
    int64_t duration = 0,
    bool is_overriding = false
);

/**
 * @brief Create a PwmCmd message with stop values (1500 for all thrusters)
 * @return PwmCmd message with all thrusters stopped
 */
crs_ros2_interfaces::msg::PwmCmd create_stop_pwm_msg();

} // namespace pwm_utils

#endif // PWM_MESSAGE_UTILS_HPP

#ifndef THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_
#define THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_

#include <string>
#include <array>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace thrust_control
{

class ThrustControlSupervisor
{
public:
	
  explicit ThrustControlSupervisor(rclcpp::Logger logger);
  void step(	  
	std::string control_mode,
       	std::array<int, 8> pwm,
	float duration,
	std::array<float, 6> error,
	std::array<float, 6> waypoint);

private:
  
  void set_control_mode(std::string control_mode);
  void set_manual_pwm(std::array<int, 8> pwm);
  void set_waypoint(std::array<float, 6> waypoint);
  void process_pwm_command(std::vector<int> pwm_array);
 
  // Control modes
  static constexpr const char* MODE_MANUAL = "manual";
  static constexpr const char* MODE_AUTONOMOUS = "autonomous";
  
  // Current control state
  std::string control_mode_;
  std::array<int, 8> current_pwm_;
  std::array<float, 6> current_waypoint_;
  
  // Logging
  rclcpp::Logger logger_;
};

}  // namespace thrust_control

#endif  // THRUST_CONTROL_THRUST_CONTROL_SUPERVISOR_HPP_

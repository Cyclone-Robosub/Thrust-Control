#include "thrust_control_node.hpp"

namespace thrust_control
{

ThrustControlNode::ThrustControlNode()
    : Node("thrust_control_node"), 
  supervisor_(this->get_logger(), nullptr)
{
  subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "array_Cltool_topic", 10, std::bind(&ThrustControlNode::topic_callback, this, std::placeholders::_1));
}

void ThrustControlNode::topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "Received PWM array with %zu values", msg->data.size());
  
  // Print the values in the array
  std::stringstream ss;
  ss << "PWM values: ";
  for (size_t i = 0; i < msg->data.size(); ++i) {
    ss << msg->data[i];
    if (i < msg->data.size() - 1) {
      ss << ", ";
    }
  }
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

}  // namespace thrust_control


void ThrustControlNode::clearQueue() {
  std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
  std::queue<std::unique_ptr<Pwm_Command>> empty;
  std::swap(ManualPWMQueue, empty);
  output << "Manual Command Current Override -> Deleted Queue"
            << std::endl;
}

// these callback functions serve as the "read Input node in the loop"
//Feel free to later push these into the Sensors Class, but make sure ThrustControlNode can still access through memory its needed fields.
void ThrustControlNode::ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
  isManualEnabled = msg->data;
  if (isManualEnabled) {
    output << "Manual Control Enabled" << std::endl;
    // need to add to here later.
  } else {
    output << "Manual Control Disabled" << std::endl;
    clearQueue();
  }
  Change_Manual.notify_all();
}


//This should only clear the queue
void ThrustControlNode::ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  std::unique_lock<std::mutex> Manual_Lock(Manual_Override_mutex);
  isManualOverride = true;
  clearQueue();
}


void ThrustControlNode::depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
  //  std::lock_guard<std::mutex> lock(mutex_);
  // std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
  // output << "Got depth ";
  depth_pressure_msg = msg->data;
}


void ThrustControlNode::imuSensorCallback(const sensor_msgs::msg::Imu &msg) {
  std::lock_guard<std::mutex> CallBacklock(imu_mutex);
  // std::cout << "imu sensor\n";
  angular_velocity_x = msg.angular_velocity.x;
  angular_velocity_y = msg.angular_velocity.y;
  angular_velocity_z = msg.angular_velocity.z;
  linear_acceleration_x = msg.linear_acceleration.x;
  linear_acceleration_y = msg.linear_acceleration.y;
  linear_acceleration_z = msg.linear_acceleration.z;
}


void ThrustControlNode::magCallback(const sensor_msgs::msg::MagneticField &msg) {
  mag_field_x = msg.magnetic_field.x;
  mag_field_y = msg.magnetic_field.y;
  mag_field_z = msg.magnetic_field.z;
}

// First get the PWM Array. Then allow the duration callback to execute and pair the array with the duration.
// Then push it onto the queue for ExecuteDecision. Notify every time we allow either Duration or Execute to
// use the queue for chain of execution. 
void ThrustControlNode::PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  output << "Received Int32MultiArray: ";
  int i = 0;
  int setvalue;
  std::unique_lock<std::mutex> pwm_lock(array_duration_sync_mutex);
  // pwm_array receivedArray;
  for (int32_t value : msg->data) {
    setvalue = (int)value;
    given_array.pwm_signals[i] = setvalue;
    output << given_array.pwm_signals[i];
    ++i;
  }
  AllowDurationSync = true;
  output << std::endl;
  SendToDuration_change.notify_all();
  pwm_lock.unlock();
}


void ThrustControlNode::durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  std::unique_lock<std::mutex> duration_lock(array_duration_sync_mutex,
                                              std::defer_lock);
  SendToDuration_change.wait(duration_lock,
                              [this] { return AllowDurationSync; });
  output << "Getting duration" << std::endl;
  auto duration_int_pwm = msg->data;
  std::unique_ptr<Pwm_Command> newCommand;
  std::unique_lock<std::mutex> Queue_sync_lock(Queue_pwm_mutex);
  switch (duration_int_pwm) {
  case -1: // PWM
    newCommand = std::make_unique<Untimed_Command>(given_array);
    ManualPWMQueue.push(newCommand);
    haltCurrentCommand();
    break;
  default: // TIMED PWM
    std::chrono::milliseconds durationMS = std::chrono::milliseconds(duration_int_pwm * 1000);
    output << durationMS << std::endl;
    newCommand = std::make_unique<Timed_Command>(given_array, durationMS);
    ManualPWMQueue.push(newCommand);
    ManualPWMQueue.push(std::make_unique<Untimed_Command>(stop_set_array));
    break;
  }
  AllowDurationSync = false;
  PWM_cond_change.notify_all();
  output << "Pushed to queue, Duration: " << duration_int_pwm << std::endl;
}



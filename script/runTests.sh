#!/bin/bash

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON

# Source the local setup
source install/setup.bash

# Run the test executable directly
./build/thrust_control/test_thrust_control_supervisor


#!/bin/bash

# run from /Thrust-Control/

# Source ROS environment
source /opt/ros/jazzy/setup.bash

 # build crs_common
 cd include/crs_common
 . ./script/build.sh
 cd ../..

# Build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON -DMOCK_RPI=ON

# Source the local setup
source install/setup.bash

# Run the test executable directly
./build/thrust_control/thrust_control_tests

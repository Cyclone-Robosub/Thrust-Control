#!/bin/bash

# run from /Thrust-Control/


#build crs_common
# cd include/crs_common
# . ./script/build.sh
# cd ../..

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Build with testing enabled
colcon build --cmake-args -DBUILD_TESTING=ON

# Source the local setup
source install/setup.bash

# Run the test executable directly
./build/thrust_control/thrust_control_tests

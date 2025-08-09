#!/bin/bash

# run from /Thrust-Control/

# Source ROS environment
source /opt/ros/jazzy/setup.bash



## build crs_common
#cd include/crs_common
#. ./script/build.sh
#if [ $? -ne 0 ]; then
#    echo "ERROR: crs_common build failed"
#    exit 1
#fi
#cd ../..

# Build with testing enabled
colcon build  --cmake-args -DBUILD_TESTING=ON -DMOCK_RPI=ON
if [ $? -ne 0 ]; then
    echo "ERROR: Main project build failed"
    exit 1
fi

# Source the local setup
source install/setup.bash

# Run the test executable directly (only if builds succeeded)
echo "Build succeeded, running tests..."
./build/thrust_control/thrust_control_tests 

#!/bin/bash

# run from /Thrust-Control/

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# build crs_common
cd include/crs_common
. ./script/build.sh
if [ $? -ne 0 ]; then
    echo "ERROR: crs_common build failed"
    exit 1
fi
cd ../..

# Build with symlink-install and mock RPI
colcon build --symlink-install --cmake-args -DMOCK_RPI=ON
if [ $? -ne 0 ]; then
    echo "ERROR: Main project build failed"
    exit 1
fi

# Source the local setup
source install/setup.bash

# Run the thrust control node
echo "Build succeeded, running thrust_control node..."
ros2 run thrust_control thrust_control

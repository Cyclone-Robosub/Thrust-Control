source /opt/ros/jazzy/setup.bash

colcon build --cmake-args -DBUILD_TESTING=ON -DMOCK_RPI=OFF
if [ $? -ne 0 ]; then
    echo "ERROR: Main project build failed"
    exit 1
fi

source install/setup.bash

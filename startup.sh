source /opt/ros/jazzy/setup.bash
colcon build

source install/setup.bash

ros2 run thrust_control thrust_control

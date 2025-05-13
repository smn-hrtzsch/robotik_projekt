#!/bin/zsh
colcon build --packages-select waymo
source install/setup.bash
ros2 run waymo keyboard_handler_node
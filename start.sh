#! /usr/bin/env bash

# Change running directory to the script directory:
cd "$(dirname "$0")"

colcon build --symlink-install;
source install/setup.bash;
ros2 run my_robot_controller farm_manager &
ros2 run my_robot_controller move_to_cell;

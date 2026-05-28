#! /usr/bin/env bash

echo "Starting the robot controller"

# Change running directory to the script directory:
cd "$(dirname "$0")"

colcon build --symlink-install;
source install/setup.bash;

# ros2 run r2drip2 test; # Can be used to run the test program (in the base.py file)
ros2 run r2drip2 farm_manager &
ros2 run r2drip2 move_to_cell;

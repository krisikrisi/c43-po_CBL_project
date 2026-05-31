#! /usr/bin/env bash

cleanup() {
  echo "Cleaning up..."
  pkill -P $$ # Kill all the processes that have the current process as a parent
}
# If someone Ctrl+C's or the process stops, make sure to run the cleanup function:
trap cleanup SIGINT SIGTERM EXIT

echo "Starting the robot controller"

# Change running directory to the script directory:
cd "$(dirname "$0")"

colcon build --symlink-install;
source install/setup.bash;

# ros2 run r2drip2 test; # Can be used to run the test program (in the base.py file)
ros2 run r2drip2 farm_manager &
ros2 run r2drip2 decision_system &
ros2 run r2drip2 move_to_cell;
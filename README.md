# R2-DRIP2

A DTAS, which simulates the environment of a farm, (finish the description) 

##  Features
- Fake weather API
- Movement (custom implementation, no libray used)
- Autonomous decision making
- Digital entity (aka the farm)

## How to run:
1. Open GAZEBO and the Docker (like they explain on canvas)
2. Run start.sh (inside the docker container)
3. Good luck

Instead of running start.sh, you can also run the following ros2 commands in parralel:
```
ros2 run r2drip2 farm_manager
ros2 run r2drip2 decision_system
ros2 run r2drip2 weather_service
ros2 run r2drip2 move_to_cell
```
(You will need to compile beforehand with ```colcon build --symlink-install; source install/setup.bash;``` if you decide to not use start.sh)

## Lab login:
Username: 
Password: 

## How to add features
1. DONT PUSH TO MAIN!!!
2. Add a branch (dont forget to publish it) with the name of your feature
3. Push your features to the branch you just created (please refer to point 1)
4. If your feature is done, create a pull reqeust to merge it to main (hopefully Hidde has time to look at it)
5. Please refer to point 1
6. Dont merge the pull request yourself, first let someone else take a look at it, preferably Hidde :D

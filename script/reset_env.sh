#!/bin/bash

# set ego states
# rosparam set /ego_initial_x 0.7
# rosparam set /ego_initial_y 0.0
# rosparam set /ego_initial_theta 0.1
rosparam set /ego_initial_x 0.0
rosparam set /ego_initial_y -1.0
rosparam set /ego_initial_theta 3.14

# set opp states
rosparam set /opp_initial_x 100.0
rosparam set /opp_initial_y 0.0
rosparam set /opp_initial_theta 1.57

# set seed to rosparam
rosparam set /seed 0

# reset gym environment
rostopic pub -1 /reset_gym_env std_msgs/Bool "data: True"

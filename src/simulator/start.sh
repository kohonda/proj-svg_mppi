#!/bin/bash

NUM_STATIC_OBS=$1

if [ -z "$NUM_STATIC_OBS" ]
then
    NUM_STATIC_OBS=5
fi

source /catkin_ws/devel/setup.bash

roslaunch f1tenth_gym_ros gym_bridge.launch num_static_obstacles:=${NUM_STATIC_OBS}
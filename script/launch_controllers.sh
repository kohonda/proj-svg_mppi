#!/bin/bash

MAP_NAME=$1
MPPI_PARAM_PATH=$2
IS_VISUALIZE=$3


if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: $0 <map_name> <mppi_param_path> <is_visualize>"
    echo "<map_name> is optional. Default is 'berlin'."
    echo "<mppi_param_path> is optional. default is src/mppi_controller/config/mppi_controller.yaml"
    echo "<is_visualize> is optional. default is true"
    exit 1
fi

if [ -z $MAP_NAME ]; then
    MAP_NAME="berlin"
fi

if [ -z "$MPPI_PARAM_PATH" ]; then
    MPPI_PARAM_PATH="default"
fi

if [ -z "$IS_VISUALIZE" ]; then
    IS_VISUALIZE=true
fi

# get location of root workspace
ROOT_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"

# source ros & root project
source /opt/ros/noetic/setup.bash;
source $ROOT_WS/devel/setup.bash;

# reset env
$ROOT_WS/script/reset_env.sh&

# launch nodes
if [ "$MPPI_PARAM_PATH" == "default" ]; then
    cd $ROOT_WS && roslaunch launch/simulation_launcher.launch workspace:=$ROOT_WS map_name:=$MAP_NAME use_rviz:=$IS_VISUALIZE
else
    cd $ROOT_WS && roslaunch launch/simulation_launcher.launch workspace:=$ROOT_WS map_name:=$MAP_NAME mppi_param_path:=$MPPI_PARAM_PATH use_rviz:=$IS_VISUALIZE
fi

#!/bin/bash

MAP_NAME=$1
MPPI_PARAM_PATH=$2
IS_VISUALIZE=$3
if [ -z "$MAP_NAME" ]; then
    echo "[ERROR] please specify map name."
    echo "Usage: $0 <map_name> <mppi_param_path> <is_visualize>"
    echo "<mppi_param_path> is optional. default is src/control/mppi_controller/config/mppi_controller.yaml"
    echo "<is_visualize> is optional. default is true"
    exit 1
fi

if [ -z "$MPPI_PARAM_PATH" ]; then
    MPPI_PARAM_PATH="default"
fi

if [ -z "$IS_VISUALIZE" ]; then
    IS_VISUALIZE=true
fi

# get location of suzlab workspace
SUZ_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"

# source ros & suzlab project
source /opt/ros/noetic/setup.bash;
source $SUZ_WS/devel/setup.bash;

# reset env
$SUZ_WS/script/reset_env.sh&

# launch nodes
if [ "$MPPI_PARAM_PATH" == "default" ]; then
    cd $SUZ_WS && roslaunch launch/simulation_launcher.launch workspace:=$SUZ_WS map_name:=$MAP_NAME use_rviz:=$IS_VISUALIZE
else
    cd $SUZ_WS && roslaunch launch/simulation_launcher.launch workspace:=$SUZ_WS map_name:=$MAP_NAME mppi_param_path:=$MPPI_PARAM_PATH use_rviz:=$IS_VISUALIZE
fi

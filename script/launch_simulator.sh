#!/bin/bash

# get gym simulator workspace
SIM_WS=$1 # recommended to use "~/sim_ws"
MAP_NAME=$2
NUM_STATIC_OBSTACLES=$3

if [ -z "$SIM_WS" ]; then
    echo "[ERROR] please specify gym simulator workspace path."
    echo "Usage: $0 <path_sim_ws> <map_name> <num_static_obstacles>"
    echo "<num_static_obstacles> is optional. Default is 5."
    exit 1
fi
if [ -z "$MAP_NAME" ]; then
    echo "[ERROR] please specify map name."
    echo "Usage: $0 <path_sim_ws> <map_name> <num_static_obstacles>"
    echo "<num_static_obstacles> is optional. Default is 5."
    exit 1
fi
if [ -z "$NUM_STATIC_OBSTACLES" ]; then
    NUM_STATIC_OBSTACLES=5
fi

# get location of suzlab workspace
SUZ_WS=$(cd $(dirname $0) && cd .. && pwd) # recommended to use "~/suzlab_ws"

# copy map data from suzlab_ws to sim_ws
map_dir=$SUZ_WS/data/map/$MAP_NAME
if [ ! -d $map_dir ]; then
    echo "[ERROR] map data does not exist: $map_dir"
    exit 1
fi
cp $SUZ_WS/data/map/$MAP_NAME/map.png  $SIM_WS/src/simulator/maps/map.png;
cp $SUZ_WS/data/map/$MAP_NAME/map.yaml $SIM_WS/src/simulator/maps/map.yaml;
cp $SUZ_WS/data/reference_path/$MAP_NAME/opp_ref_path.csv $SIM_WS/src/simulator/maps/opp_ref_path.csv

# launch ros1 simulator nodes (rviz)
echo "[INFO] launch gym simulator";
gnome-terminal --title="gym_ros1_wrapper" -- bash -c "source /opt/ros/noetic/setup.bash; source $SIM_WS/devel/setup.sh; roslaunch f1tenth_gym_ros agent_template.launch" \
    && sleep 1s;

# cleaning up in advance
DOCKER_STATUS=`docker inspect --format='{{.State.Status}}' f1tenth_gym_container`;
if [ $DOCKER_STATUS == "running" ]; then
    echo "[INFO] launch gym environment on a docker container.";
    docker rm -f f1tenth_gym_container;
fi

# setup gym environment
echo "[INFO] build and launch gym environment on a docker container.";
cd $SIM_WS/src/simulator && $SIM_WS/src/simulator/build_docker.sh&& $SIM_WS/src/simulator/docker.sh $NUM_STATIC_OBSTACLES;

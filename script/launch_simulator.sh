#!/bin/bash

MAP_NAME=$1
NUM_STATIC_OBSTACLES=$2

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: $0 <map_name> <num_static_obstacles>"
    echo "<map_name> is optional. Default is 'berlin'."
    echo "<num_static_obstacles> is optional. Default is 5."
    exit 1
fi

if [ -z $MAP_NAME ]; then
    MAP_NAME="berlin"
fi

if [ -z "$NUM_STATIC_OBSTACLES" ]; then
    NUM_STATIC_OBSTACLES=5
fi

# get location of root workspace
ROOT_WS=$(cd $(dirname $0) && cd .. && pwd) 

# copy map data from root to ROOT_WS
map_dir=$ROOT_WS/data/map/$MAP_NAME
if [ ! -d $map_dir ]; then
    echo "[ERROR] map data does not exist: $map_dir"
    exit 1
fi
cp $ROOT_WS/data/map/$MAP_NAME/map.png  $ROOT_WS/src/simulator/maps/map.png;
cp $ROOT_WS/data/map/$MAP_NAME/map.yaml $ROOT_WS/src/simulator/maps/map.yaml;
cp $ROOT_WS/data/reference_path/$MAP_NAME/opp_ref_path.csv $ROOT_WS/src/simulator/maps/opp_ref_path.csv

# launch simulator nodes 
echo "[INFO] launch gym simulator";
gnome-terminal --title="gym_ros1_wrapper" -- bash -c "source /opt/ros/noetic/setup.bash; source $ROOT_WS/devel/setup.sh; roslaunch f1tenth_gym_ros agent_template.launch" \
    && sleep 1s;

# cleaning up in advance
DOCKER_STATUS=`docker inspect --format='{{.State.Status}}' f1tenth_gym_container`;
if [ $DOCKER_STATUS == "running" ]; then
    echo "[INFO] launch gym environment on a docker container.";
    docker rm -f f1tenth_gym_container;
fi

# setup gym environment
echo "[INFO] build and launch gym environment on a docker container.";
cd $ROOT_WS/src/simulator && $ROOT_WS/src/simulator/build_docker.sh&& $ROOT_WS/src/simulator/docker.sh $NUM_STATIC_OBSTACLES;

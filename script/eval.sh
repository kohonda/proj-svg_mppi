#!/bin/bash

# check installed xdotool
if ! type "xdotool" > /dev/null 2>&1; then
    echo "[ERROR] xdotool is not installed. Please install xdotool."
    exit 1
fi

MAP_NAME="berlin"
EVAL_NAME=$1
MPPI_PARAM_PATH=$2
NUM_TRIALS=$3
NUM_STATIC_OBSTACLES=$4
IS_VISUALIZE=$5

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: $0 <eval_name> <mppi_param_path> <num_trials> <num_static_obstacles> <is_visualize>"
    echo "<eval_name> is optional. default is 'default'"
    echo "<mppi_param_path> is optional. default is src/mppi_controller/config/mppi_controller.yaml"
    echo "<num_trials> is optional. default is 100"
    echo "<num_static_obstacles> is optional. default is 5"
    echo "<is_visualize> is optional. default is true"
    exit 0
fi

if [ -z "$EVAL_NAME" ]; then
    EVAL_NAME="default"
fi

if [ -z "$MPPI_PARAM_PATH" ]; then
    MPPI_PARAM_PATH="default"
fi

if [ -z "$NUM_TRIALS" ]; then
    NUM_TRIALS=100
fi

if [ -z "$NUM_STATIC_OBSTACLES" ]; then
    NUM_STATIC_OBSTACLES=5
fi

if [ -z "$IS_VISUALIZE" ]; then
    IS_VISUALIZE=false
fi


# run simulator
gnome-terminal --title="ros_gym_simulator" -- bash -c "./launch_simulator.sh $MAP_NAME $NUM_STATIC_OBSTACLES";
# wait for simulator to be ready: check xdotool
while [ -z "$(xdotool search --name "/catkin_ws/src/f1tenth_gym_ros/launch/gym_bridge.launch http://localhost:11311")" ]; do
    echo "[INFO] waiting for simulator to be ready..."
    sleep 3s;
done

# run controllers
gnome-terminal --title="controllers" -- bash -c "./launch_controllers.sh $MAP_NAME $MPPI_PARAM_PATH $IS_VISUALIZE" \
    && sleep 3s;

# run evaluation node
ROOT_WS=$(cd $(dirname $0) && cd .. && pwd)
source /opt/ros/noetic/setup.bash;
source $ROOT_WS/devel/setup.bash;
roslaunch eval_local_planner eval.launch trial_num:=$NUM_TRIALS eval_name:=$EVAL_NAME
pid_eval=$!
echo "[INFO] evaluation pid: $pid_eval"
wait $pid_eval
echo "[INFO] evaluation finished"

# kill simulator of another terminal
docker rm -f f1tenth_gym_container

# kill controllers
xdotool windowkill $(xdotool search --name "launch/simulation_launcher.launch http://localhost:11311");


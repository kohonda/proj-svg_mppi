#!/bin/bash

NUM_STATIC_OBS=$1

if [ -z "$NUM_STATIC_OBS" ]
then
    NUM_STATIC_OBS=5
fi

docker run -it --name=f1tenth_gym_container --rm --net=host f1tenth_gym $NUM_STATIC_OBS


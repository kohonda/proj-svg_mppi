#!/bin/bash
if [ ! -d f1tenth_gym ] ; then
    git clone git@github.com:IV2023ADContest/f1tenth_gym.git
else
    echo f1tenth_gym exists, not cloning.
fi
docker build -t f1tenth_gym -f Dockerfile .;
docker image prune --force;
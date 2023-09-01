# Usage: make [command]
SHELL := /bin/bash
setup:
	# install dependencies for ROS 	
	rosdep install -i --from-path src --rosdistro noetic -y
	sudo apt update
	sudo apt install -y ros-noetic-costmap-2d ros-noetic-map-server ros-noetic-gmapping ros-noetic-amcl python3-catkin-tools libomp-dev ros-noetic-jsk-rviz-plugins ros-noetic-urg-node mpv

# build without simulator with ROS
.PHONY: build
build:
	. /opt/ros/noetic/setup.sh && \
	LOCAL_IP=`hostname -I | cut -d' ' -f1` && \
	# export ROS_IP=${LOCAL_IP} && \
	# export ROS_MASTER_URI=http://${LOCAL_IP}:11311 && \
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_GPU=OFF

build-cuda:
	. /opt/ros/noetic/setup.sh && \
	LOCAL_IP=`hostname -I | cut -d' ' -f1` && \
	export ROS_IP=${LOCAL_IP} && \
	export ROS_MASTER_URI=http://${LOCAL_IP}:11311 && \
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_GPU=ON 

build-debug:
	. /opt/ros/noetic/setup.sh && \
	LOCAL_IP=`hostname -I | cut -d' ' -f1` && \
	export ROS_IP=${LOCAL_IP} && \
	export ROS_MASTER_URI=http://${LOCAL_IP}:11311 && \
	catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
	# clean build files
	rm -rf build devel logs .catkin_tools install

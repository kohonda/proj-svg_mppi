FROM ros:noetic-ros-base-focal

ENV IM_IN_DOCKER Yes

RUN apt-get update --fix-missing && \
    apt-get install -y \
                    python3-dev \
                    python3-pip \
                    python-is-python3 \
                    git \
                    build-essential \
                    libgl1-mesa-dev \
                    mesa-utils \
                    libglu1-mesa-dev \
                    fontconfig \
                    libfreetype6-dev

RUN apt-get install -y ros-noetic-ackermann-msgs \
                       ros-noetic-map-server \
                       ros-noetic-genpy \
                       ros-noetic-xacro \
                       ros-noetic-robot-state-publisher 

# RUN pip3 install --upgrade pip

RUN pip3 install PyOpenGL \
                 PyOpenGL_accelerate


# RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN mkdir /f1tenth_gym
COPY ./f1tenth_gym /f1tenth_gym

RUN cd f1tenth_gym && \
    pip3 install -e .

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; mkdir -p catkin_ws/src; cd catkin_ws; catkin_make"

RUN mkdir /catkin_ws/src/f1tenth_gym_ros

COPY . /catkin_ws/src/f1tenth_gym_ros

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd catkin_ws; catkin_make; source devel/setup.bash"

# CMD ["/catkin_ws/src/f1tenth_gym_ros/start.sh"]
ENTRYPOINT [ "/catkin_ws/src/f1tenth_gym_ros/start.sh" ]

# CMD ["roslaunch", "package file.launch"]

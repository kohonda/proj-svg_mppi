FROM ros:noetic-ros-base-focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV DISABLE_ROS1_EOL_WARNINGS=true

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    python3-catkin-tools \
    python-is-python3 \
    wget \
    curl \
    sudo \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-grid-map \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-msgs \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    ros-${ROS_DISTRO}-grid-map-visualization \
    ros-${ROS_DISTRO}-grid-map-pcl \
    ros-${ROS_DISTRO}-costmap-2d \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-jsk-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install additional dependencies from Makefile
RUN apt-get update && apt-get install -y \
    libomp-dev \
    mpv \
    xdotool \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    matplotlib \
    pandas \
    PyYAML

# Setup workspace
WORKDIR /workspace

# Initialize rosdep
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update"

# Create workspace directory
RUN mkdir -p /workspace

# Setup entrypoint
RUN echo "#!/bin/bash\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
if [ -f /workspace/devel/setup.bash ]; then\n\
    source /workspace/devel/setup.bash\n\
fi\n\
exec \"\$@\"" > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Enable GUI support
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Create a non-root user that can be overridden at runtime
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd -g ${GROUP_ID} developer && \
    useradd -m -u ${USER_ID} -g developer -s /bin/bash developer && \
    echo "developer ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Make entrypoint executable by all users
RUN chmod 755 /ros_entrypoint.sh

# Set working directory permissions
RUN chmod -R 755 /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
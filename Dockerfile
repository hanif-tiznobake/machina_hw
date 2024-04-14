# Base Image
FROM ros:humble-ros-core-jammy

# Install development tools and dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# Setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Your ROS2 workspace directory
WORKDIR /ros2_ws

# Copy your local ROS2 package into the Docker container
COPY ./src /ros2_ws/src

# Install any Python dependencies
RUN pip3 install numpy scipy

# Resolve dependencies using rosdep
RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the ROS2 packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Set the entrypoint to source the ROS2 setup script
ENTRYPOINT ["/ros2_ws/install/setup.bash"]

# Command for keeping the container running for development
CMD ["tail", "-f", "/dev/null"]

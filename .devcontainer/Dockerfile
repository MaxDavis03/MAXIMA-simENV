FROM ros:humble-ros-base

# Install dependencies
RUN apt update && apt install -y \
    python3-pip \
    git \
    build-essential \
    cmake \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python ROS tools
RUN pip3 install -U \
    empy \
    colcon-common-extensions \
    catkin_pkg \
    lark-parser

# Setup micro-ROS build system
RUN mkdir -p /root/microros_ws/src
WORKDIR /root/microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Setup ROS2 simulation workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Optional: copy simulation code in
# COPY ./your-simulation-code ./src

# Build all
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Entry point
CMD ["/bin/bash"]

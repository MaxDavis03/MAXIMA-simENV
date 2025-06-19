# ---------- Base Image ----------
FROM ros:humble-ros-base AS base

# Install common dependencies and ROS bridges
RUN apt update && apt install -y \
    python3-pip \
    git \
    build-essential \
    cmake \
    libusb-1.0-0-dev \
    curl \
    wget \
    nano \
    python3-serial \
    python3-pyparsing \
    python3-pyelftools \
    gcc \
    gcc-arm-none-eabi \
    ninja-build \
    unzip \
    python3-colcon-common-extensions \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Python-based ROS tools
RUN pip3 install -U \
    empy \
    colcon-common-extensions \
    catkin_pkg \
    lark-parser

# ---------- SIM ENV ----------
FROM base AS simenv

# Setup micro-ROS build system
RUN mkdir -p /root/microros_ws/src
WORKDIR /root/microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Setup ROS2 simulation workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws
# COPY ./MAXIMAsim ./src  # (optional) simulation packages

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Auto-start agent + bridges
CMD ["/bin/bash", "-c", "\
  source /opt/ros/humble/setup.bash && \
  source /root/microros_ws/install/local_setup.bash && \
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 & \
  ros2 launch launch_simenv.launch.xml \
"]

# ---------- DEV ENV ----------
FROM base AS devenv

# Install VS Code Server (headless IDE)
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Setup config.yaml
RUN mkdir -p /root/.config/code-server
#COPY --chown=root:root ./config.yaml /root/.config/code-server/config.yaml

# Setup workspaces
RUN mkdir -p /root/microros_ws/src /root/ros2_ws/src

WORKDIR /root/microros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

EXPOSE 8080
CMD ["code-server", "--bind-addr", "0.0.0.0:8080", "/root"]

# ---------- MICRO-ROS DEV ENV ----------
FROM base AS microrosdev

# Create workspace
RUN mkdir -p /root/microros_ws/src
WORKDIR /root/microros_ws

# Clone micro-ROS setup
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Preinstall ESP-IDF (v4.4.5)
ENV IDF_VERSION=v4.4.5
ENV IDF_PATH=/opt/esp/idf

RUN mkdir -p /opt/esp && \
    cd /opt/esp && \
    git clone -b ${IDF_VERSION} --recursive https://github.com/espressif/esp-idf.git idf && \
    /opt/esp/idf/install.sh

ENV PATH="${IDF_PATH}/tools:$PATH"

CMD ["/bin/bash"]

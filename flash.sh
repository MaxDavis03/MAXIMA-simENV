#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd ~/microros_ws
source install/local_setup.bash

# Setup ESP32 build system
ros2 run micro_ros_setup create_firmware_ws.sh esp32
ros2 run micro_ros_setup build_firmware.sh

# Flash device
PORT=${1:-/dev/ttyUSB0}
echo "Flashing micro-ROS firmware to $PORT"
ros2 run micro_ros_setup flash_firmware.sh $PORT

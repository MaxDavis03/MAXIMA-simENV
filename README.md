# MAXIMA-simENV
# MAXIMA-simENV & devENV Docker Setup
Updated 19/06/2025

## Overview

This repository contains a multi-purpose Docker setup for:

* **Simulation environment** (`maxima-simenv`) with ROS 2 Humble, micro-ROS, rosbridge, Foxglove bridge (auto-started), and micro-ROS Agent
* **Development environment** (`maxima-devenv`) with code-server (VS Code in browser)
* **Firmware build environment** (`maxima-microrosdev`) with ESP-IDF 4.4.5 and cross-toolchains

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR-USERNAME/MAXIMA-simENV.git
cd MAXIMA-simENV
```

### 2. Build and Launch with Docker Compose

```bash
docker compose -f .devcontainer/docker-compose.yml up --build
```

### 3. Open VS Code Dev Container

* Install the **Dev Containers** extension in VS Code
* Open folder in container (`.devcontainer` will auto-configure)

### 4. micro-ROS Agent Example (UDP or Serial)

The `maxima-simenv` container auto-starts the UDP agent. You can also run manually:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

```bash
# Or for serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### 5. ESP32 Firmware Build Example

```bash
source /opt/ros/humble/setup.bash
source /root/microros_ws/install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh esp32
ros2 run micro_ros_setup build_firmware.sh
ros2 run micro_ros_setup flash_firmware.sh /dev/ttyUSB0
```

### 6. Flash Script (optional)

Use the provided flash helper:

```bash
chmod +x flash.sh
./flash.sh /dev/ttyUSB0
```

### 7. Launch File for Bridges

Use this to launch bridges manually inside simENV:

```bash
ros2 launch launch_simenv.launch.xml
```

---

## Services Summary

| Service Name         | Purpose                              | Access                  |
| -------------------- | ------------------------------------ | ----------------------- |
| `maxima-simenv`      | ROS 2 + Foxglove + rosbridge + agent | Internal                |
| `maxima-devenv`      | Code editor in browser               | `http://localhost:8080` |
| `maxima-microrosdev` | micro-ROS firmware builds            | Internal                |

---

## License

MIT



















###OUTDATED##################################################
Installation instructions for MAXIMA simulation environment.


Updated 06 JUN 2025

Starting with a fresh install of Ubunutu 22.04 Desktop (Jammy):
You will require an internet connection and Sudo privileges.

In the terminal, run:

#===============SYSTEM===============#
#ensure default packages are up to date after fresh install
sudo apt update
sudo apt upgrade -y


#===============DOCKER===============#
#install and configure docker
sudo apt install -y docker.io
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker $USER
newgrp docker
#you may need to log out and back in after this step


#===============ROS 2 HUMBLE===============#
#setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

#install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

#source ROS 2 in your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


#===============MICRO-ROS AGENT===============#
#pull the micro-ROS Agent image
sudo apt update
docker pull microros/micro-ros-agent:humble

#setup micro-ROS Agent as a persistent service
docker run -d \
  --name micro-ros-agent \
  --restart=unless-stopped \
  --network=host \
  microros/micro-ros-agent:humble \
  udp4 --port 8888
--restart=unless-stopped

#verify micro-ROS Agent is running
docker ps
docker logs micro-ros-agent

#you should see something like:
running... | port: 8888
logger setup | verbose_level: 4

#add support via UART for non-networked MCUs
micro_ros_agent serial --dev /dev/ttyUSB0  #++++++++++++NOT WORKING++++++++++++


#===============UNITY===============#
#follow the official guide to install Unity for Ubuntu from https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux
#it is recommended to use LTS version 2021.3 or 2022.3 for compatability with ROS

#or follow the list of instructions below:
    #To install the Unity Hub on a Debian or Ubuntu Linux distribution, you need to add the Unity Hub Debian         repository along with the public signing key to verify the integrity of the packages.
    #To add the public signing key, run the following command:
    wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
    #To add the Unity Hub repository, you need an entry in /etc/apt/sources.list.d. Run the following command to     add the Unity Hub repository:
    sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
    #Update the package cache and install the package:
    sudo apt update
    sudo apt-get install unityhub
    #**Note: For some systems, you may need to ensure the following:
      #The directory /usr/share/keyrings exists.
      #The user or group installing the Hub has write permissions to the /usr/share/keyrings directory.
      #The user or group installing Hub has at least read permissions to the resulting file                 Unity_Technologies_ApS.gpg.**

#then run Unity Hub with command:
unityhub

#when Unity Hub opens, skip past the recommended Unity Editor installation and install either version 2021.3 LTS or 2022.3 LTS, and make sure to check 'Linux Dedicated Server Build Support' or any other packages you may want to use when building your application for distribution 

#Once Unity Editor is installed, you can use the prebuilt MAXIMA simulation template:
git clone https://github.com/max_davis03/MAXIMA-simENV/MAXIMA-StandardTemplate.git
#Then, in Unity Hub:
    #Click "Open" and navigate to the cloned project folder
    #Ensure you are using Unity version 2021.3 LTS or 2022.3 LTS
    #ROS settings should already be configured for use with rosbridge (port 9090 by default)


#++++++++++++++++++IF YOU ARE CONFIGURING YOUR OWN PROJECT WITHOUT THE TEMPLATE+++++++++++++++++

#in your Unity project, install the ROS-TCP-Connector Unity Package in the Unity package manager, or follow the guide: https://github.com/Unity-Technologies/ROS-TCP-Connector

#add the rosbridge_server in the ROS 2 environment:
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

#then in Unity:
#set the ROS IP and port(default:9090)
#use the TCP Connector or ROSConnection script to link your nodes
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#===============FOXGLOVE BRIDGE===============#
#install the Foxglove WebSocket bridge (C++ implementation)
sudo apt update
sudo apt install -y ros-humble-foxglove-bridge

#start the bridge with default port 8765
ros2 launch foxglove_bridge foxglove_bridge_launch.xml




RESULT:

Your micro-ROS Agent is now running at boot and listening on UDP port 8888.
You can now flash micro-ROS firmware onto MCU devices with the transport settings:
Agent IP = your machine IP
Port = 8888
Transport = UDPv4

Foxglove can connect via WebSocket (port 8765)

Unity can interact with ROS 2 topics through rosbridge_server

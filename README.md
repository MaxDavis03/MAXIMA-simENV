# MAXIMA-simENV
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
micro_ros_agent serial --dev /dev/ttyUSB0


#===============FOXGLOVE BRIDGE===============#
#install foxglove rosbridge
sudo apt install npm
sudo npm install -g @foxglove/rosbridge

#start the Websocket bridge (over port 8765 by default)
rosbridge


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

    #To remove the Unity Hub from the system, run the following command:
    sudo apt-get remove unityhub


#in your Unity project, install the ROS-TCP-Connector Unity Package in the Unity package manager, or follow the guide: https://github.com/Unity-Technologies/ROS-TCP-Connector

#add the rosbridge_server in the ROS 2 environment:
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

#then in Unity:
#set the ROS IP and port(default:9090)
#use the TCP Connector or ROSConnection script to link your nodes

#Once your Unity environment is configured, you can use the prebuilt MAXIMA simulation template:
git clone https://github.com/YOUR_USERNAME/MAXIMA-UnityTemplate.git
#Then, in Unity Hub:
    #Click "Open" and navigate to the cloned project folder
    #Ensure you are using Unity version 2021.3 LTS or 2022.3 LTS
    #ROS settings should already be configured for use with rosbridge (port 9090 by default)


RESULT:

Your micro-ROS Agent is now running at boot and listening on UDP port 8888.
You can now flash micro-ROS firmware onto MCU devices with the transport settings:
Agent IP = your machine IP
Port = 8888
Transport = UDPv4

Foxglove can connect via WebSocket (port 8765)

Unity can interact with ROS 2 topics through rosbridge_server

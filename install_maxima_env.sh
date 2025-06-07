#!/bin/bash

echo "========== MAXIMA Simulation Environment Installer =========="

#---------------------------------------------
# SYSTEM PREPARATION
#---------------------------------------------
echo "[1/10] Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo "[2/10] Installing required packages..."
sudo apt install -y curl gnupg lsb-release docker.io npm unzip gnome-terminal

#---------------------------------------------
# DOCKER CONFIGURATION
#---------------------------------------------
echo "[3/10] Enabling and starting Docker..."
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker $USER

#---------------------------------------------
# ROS 2 HUMBLE INSTALLATION
#---------------------------------------------
echo "[4/10] Setting up ROS 2 Humble sources..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[5/10] Installing ROS 2 Humble Desktop..."
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-rosbridge-server ros-humble-foxglove-bridge

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

#---------------------------------------------
# UNITY HUB + EDITOR INSTALLATION
#---------------------------------------------
echo "[6/10] Installing Unity Hub..."
wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg > /dev/null
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
sudo apt update
sudo apt install -y unityhub

echo "[7/10] Installing Unity Editor (LTS)..."
unityhub -- --headless install --version 2022.3.21f1 --changeset 8e603399ca3d --module linux-il2cpp

#---------------------------------------------
# UNITY PROJECT TEMPLATE
#---------------------------------------------
echo "[8/10] Downloading Unity template project..."
mkdir -p ~/Unity/Hub/Projects/MAXIMA-UnityTemplate
curl -L https://github.com/YOUR_USERNAME/YOUR_REPO/releases/latest/download/MAXIMA-UnityTemplate.zip -o /tmp/MAXIMA-UnityTemplate.zip
unzip -o /tmp/MAXIMA-UnityTemplate.zip -d ~/Unity/Hub/Projects/MAXIMA-UnityTemplate
rm /tmp/MAXIMA-UnityTemplate.zip

#---------------------------------------------
# MICRO-ROS AGENT SETUP (DOCKER)
#---------------------------------------------
echo "[9/10] Setting up Micro-ROS Agent container..."
docker pull microros/micro-ros-agent:humble

docker run -d \
  --name micro-ros-agent \
  --restart=unless-stopped \
  --network=host \
  microros/micro-ros-agent:humble \
  udp4 --port 8888

#---------------------------------------------
# POST-INSTALL LAUNCH + INSTRUCTIONS
#---------------------------------------------
echo "[10/10] Creating and launching startup script..."
chmod +x ./maxima_startup.sh
gnome-terminal -- ./maxima_startup.sh

echo ""
echo "✅ INSTALL COMPLETE!"
echo "Next steps:"
echo "- Unity Editor has been installed. Open Unity Hub and the MAXIMA-UnityTemplate will appear."
echo "- Sign into Foxglove Studio and connect via WebSocket to ws://localhost:8765"
echo "- Run your Unity simulation or configure your own."
echo "- The startup script has launched in a new terminal to manage running services."

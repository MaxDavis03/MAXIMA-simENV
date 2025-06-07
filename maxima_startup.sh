#!/bin/bash

clear
echo "========== MAXIMA Simulation Environment Startup =========="

# Check & Start Services
echo "Checking service status..."

# Micro-ROS Agent
if ! docker ps | grep -q micro-ros-agent; then
    echo "[STARTING] Micro-ROS Agent..."
    docker start micro-ros-agent > /dev/null
else
    echo "[OK] Micro-ROS Agent already running."
fi

# Foxglove Bridge
if ! pgrep -f foxglove_bridge > /dev/null; then
    echo "[STARTING] Foxglove Bridge..."
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
    sleep 2
else
    echo "[OK] Foxglove Bridge already running."
fi

# rosbridge_server
if ! pgrep -f rosbridge_websocket > /dev/null; then
    echo "[STARTING] rosbridge_server..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    sleep 2
else
    echo "[OK] rosbridge_server already running."
fi

echo ""
# Interactive UI
while true; do
    echo "========== Component Control =========="
    echo "1) Stop Micro-ROS Agent"
    echo "2) Stop Foxglove Bridge"
    echo "3) Stop rosbridge_server"
    echo "4) Restart All"
    echo "5) Exit"
    echo "======================================="
    read -rp "Choose an option: " choice

    case $choice in
        1) docker stop micro-ros-agent ;;
        2) pkill -f foxglove_bridge ;;
        3) pkill -f rosbridge_websocket ;;
        4)
            docker restart micro-ros-agent
            pkill -f foxglove_bridge
            pkill -f rosbridge_websocket
            sleep 1
            ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
            ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
            ;;
        5) echo "Exiting UI..."; break ;;
        *) echo "Invalid option" ;;
    esac
done

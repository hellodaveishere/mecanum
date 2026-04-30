#!/usr/bin/env bash

echo "[BASH] Avvio ROS2 mecanum_base..."

# Carica ambiente ROS2
source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

# Sostituisci il processo bash con ros2 launch
# In questo modo:
#  - il PID visto da Node.js diventa il python3 di ros2 launch
#  - SIGINT da Node.js va direttamente a ros2 launch
exec ros2 launch mecanum_base bringup.launch.py

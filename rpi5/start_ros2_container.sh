#!/bin/bash

set -e

USE_GUI=false
OPEN_SHELL=false
RUN_LAUNCH=false

for arg in "$@"; do
  case $arg in
    --gui) USE_GUI=true ;;
    --shell) OPEN_SHELL=true ;;
    --launch) RUN_LAUNCH=true ;;
  esac
done

if $USE_GUI; then
  CONTAINER_NAME="ros2-dev-gui"
  PROFILE="--profile gui"
  echo "▶️ Avvio container ROS 2 con GUI..."
else
  CONTAINER_NAME="ros2-dev"
  PROFILE=""
  echo "▶️ Avvio container ROS 2 headless..."
fi

# 🔥 Soluzione standard: build solo se necessario
docker compose $PROFILE build

# Avvia container senza ricrearlo se non serve
docker compose $PROFILE up -d "$CONTAINER_NAME"

sleep 1

if $OPEN_SHELL; then
  echo "🔧 Entrando nel container $CONTAINER_NAME ..."
  docker exec -it "$CONTAINER_NAME" bash --login
  exit 0
fi

if $RUN_LAUNCH; then
  echo "🚀 Avvio ROS 2 launch nel container $CONTAINER_NAME ..."
  docker exec -it "$CONTAINER_NAME" bash --login -c \
    "source /opt/ros/jazzy/setup.bash && ros2 launch mecanum_base bringup.launch.py"
  exit 0
fi

echo "✅ Container $CONTAINER_NAME avviato."


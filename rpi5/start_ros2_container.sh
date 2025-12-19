#!/bin/bash
# ============================================================
# 🛠️ Script di avvio ROS 2 Dev Container (Headless/GUI)
# ============================================================

# 📖 ISTRUZIONI D'USO:
#
# 1. Avvio container senza GUI (headless):
#      ./start_ros2_container.sh
#
# 2. Avvio container con GUI:
#      ./start_ros2_container.sh --gui
#
# 3. Per fermare i container:
#      ./stop_ros2_container.sh
#
# 4. Per entrare nel container ROS 2:
#      docker exec -it ros2_ws_advancedbot-ros2-dev-1 bash
#
# 5. Per lanciare ROS 2 (esempio):
#      docker exec -it ros2_ws_advancedbot-ros2-dev-1 \
#        ros2 launch mecanum_base bring-up.launch.py
#
# ⚠️ Note:
# - Lo script usa `docker compose` (Compose V2).
# - La GUI richiede DISPLAY e /tmp/.X11-unix funzionanti.
# ============================================================

set -e

USE_GUI=false
RUN_SHELL=false
RUN_LAUNCH=false

# Opzioni:
#   --gui       → avvia GUI
#   --shell     → entra nel container dopo l'avvio
#   --launch    → lancia ros2 launch automaticamente

for arg in "$@"; do
  case $arg in
    --gui) USE_GUI=true ;;
    --shell) RUN_SHELL=true ;;
    --launch) RUN_LAUNCH=true ;;
  esac
done

if $USE_GUI; then
  echo "▶️ Avvio container ROS 2 con GUI..."
  docker compose --profile gui up -d --build
else
  echo "▶️ Avvio container ROS 2 headless..."
  docker compose up -d --build
fi

CONTAINER_NAME=$(docker ps --format "{{.Names}}" | grep ros2-dev)

# ✅ (1) entra nel container
if $RUN_SHELL; then
  echo "🔧 Entrando nel container..."
  docker exec -it "$CONTAINER_NAME" bash
  exit 0
fi

# ✅ (2) lancia ros2 launch
if $RUN_LAUNCH; then
  echo "🚀 Avvio ROS 2 launch..."
  docker exec -it "$CONTAINER_NAME" \
    ros2 launch mecanum_base bringup.launch.py
  exit 0
fi

echo "✅ Container avviato."


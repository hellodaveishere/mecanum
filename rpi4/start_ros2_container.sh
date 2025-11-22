#!/bin/bash
# ============================================================
# 🛠️ Script di avvio ROS 2 Dev Container (Headless/GUI)
# ============================================================
# 📖 ISTRUZIONI D'USO:
# 1. Assicurati di trovarti nella cartella del progetto:
#      cd ros2-dev-container
#
# 2. Avvio container senza GUI (headless):
#      ./start_ros2_container.sh
#
#    ➡️ Questo avvierà il servizio "ros2-dev" definito in docker-compose.yaml
#       con profilo "default". È pensato per lavorare da terminale su Raspberry Pi 4.
#
# 3. Avvio container con GUI:
#      ./start_ros2_container.sh --gui
#
#    ➡️ Questo avvierà il servizio "ros2-dev-gui" con profilo "gui".
#       Richiede che il server X sia attivo e accessibile (DISPLAY e /tmp/.X11-unix).
#
# 4. Per fermare il container:
#      ./stop_ros2_container.sh
#
# ⚠️ Note:
# - Lo script utilizza `docker compose` (non `docker-compose`).
# - Assicurati che Docker e Docker Compose siano installati e funzionanti.
# - La GUI è opzionale e può essere usata solo se il Raspberry Pi ha un ambiente grafico configurato.
# ============================================================

set -e

PROFILE="default"

if [[ "$1" == "--gui" ]]; then
  PROFILE="gui"
  echo "▶️ Avvio container ROS 2 con GUI..."
else
  echo "▶️ Avvio container ROS 2 headless..."
fi

docker compose up -d --build --profile "$PROFILE"
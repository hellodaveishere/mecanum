#!/bin/bash
# ============================================================
# 🛑 Script di stop ROS 2 Dev Container
# ============================================================
# 📖 ISTRUZIONI D'USO:
# 1. Assicurati di trovarti nella cartella del progetto:
#      cd ros2-dev-container
#
# 2. Per fermare i container avviati (sia headless che GUI):
#      ./stop_ros2_container.sh
#
#    ➡️ Questo comando fermerà e rimuoverà i container "ros2-dev"
#       e "ros2-dev-gui" se attivi.
#
# ⚠️ Note:
# - Lo script utilizza `docker compose` (non `docker-compose`).
# - Non è necessario specificare profili: vengono fermati tutti
#   i container definiti nel file docker-compose.yaml.
# ============================================================

set -e

echo "🧼 Arresto e rimozione dei container ROS 2..."
docker compose down
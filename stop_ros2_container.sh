#!/bin/bash
# ============================================================
# 🛑 Script di stop ROS 2 Dev Containers (Headless + GUI)
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

CONTAINERS=(
  "ros2-dev"
  "ros2-dev-gui"
)

for C in "${CONTAINERS[@]}"; do
  if docker ps --format "{{.Names}}" | grep -q "^${C}$"; then
    echo "🛑 Fermando il container ${C} ..."
    docker stop "$C" >/dev/null
    echo "🧹 Rimuovendo il container ${C} ..."
    docker rm "$C" >/dev/null
    echo "✅ ${C} fermato e rimosso."
  else
    echo "ℹ️ Il container '${C}' non è in esecuzione."
  fi
done


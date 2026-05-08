#!/bin/bash

# ============================================================
# 🔄 Sincronizzazione automatizzata progetto ROS 2 → Raspberry Pi 5
# ============================================================
#
# Questo script sincronizza in modo affidabile e non interattivo
# (grazie all’autenticazione SSH tramite chiavi) il progetto ROS 2
# sviluppato sul laptop verso il Raspberry Pi 5.
#
# ✔️ Funzionalità principali:
#   • Mirror completo dei pacchetti ROS 2:
#       - Sorgente:      ../src/
#       - Destinazione:  /home/dave/ws/src/
#       - Il Raspberry Pi 5 diventa una copia IDENTICA del laptop
#
#   • Mirror completo dei file Docker:
#       - Sorgente:      ./ (directory rpi5)
#       - Destinazione:  /home/dave/ros2-dev-container/
#
#   • Creazione directory remote solo se mancanti
#
#   • Gestione container ros2-dev:
#       - Se NON attivo → chiedere all’utente se avviarlo
#       - Se attivo → chiedere all’utente se riavviarlo
#
#   • Sincronizzazione tramite rsync:
#       - Modalità archivio (-a)
#       - Compressione (-z)
#       - Sovrascrittura file vecchi
#       - Eliminazione file obsoleti (--delete)
#       - Nessuna richiesta di password (SSH con chiavi)
#
# ============================================================


# 📍 Parametri fissi
RPI_USER="dave"
RPI_HOST="192.168.188.51"
#RPI_HOST="10.42.0.1"
RPI="$RPI_USER@$RPI_HOST"

SRC_DIR="../src"
RPI5_DIR="."
TARGET_DIR="/home/dave/ws/src"
DOCKER_TARGET_DIR="/home/dave/ros2-dev-container"

PACKAGES=("mecanum_base" "sllidar_ros2")
DOCKER_FILES=("compose.yaml" "Dockerfile" "start_ros2_container.sh" "stop_ros2_container.sh")


# ============================================================
# 🛠️ Creazione directory remote (solo se mancanti)
# ============================================================
echo "📁 Verifica directory remote..."

ssh "$RPI" "mkdir -p $TARGET_DIR"
ssh "$RPI" "mkdir -p $DOCKER_TARGET_DIR"


# ============================================================
# 📦 Mirror file Docker (sovrascrive + elimina file obsoleti)
# ============================================================
echo "📤 Mirror file Docker verso $DOCKER_TARGET_DIR..."

rsync -az --delete "$RPI5_DIR/" "$RPI:$DOCKER_TARGET_DIR/" \
  --include="compose.yaml" \
  --include="Dockerfile" \
  --include="start_ros2_container.sh" \
  --include="stop_ros2_container.sh" \
  --exclude="*" 


# ============================================================
# 🔍 Controllo stato container ros2-dev (con conferma utente)
# ============================================================
echo "🔎 Controllo container ros2-dev su $RPI_HOST..."

CONTAINER_STATUS=$(ssh "$RPI" docker ps --filter "name=ros2-dev" --filter "status=running" --format "{{.Names}}")

if [[ "$CONTAINER_STATUS" != "ros2-dev" ]]; then
  echo "⚠️ Il container ros2-dev NON è attivo."
  read -p "👉 Vuoi avviarlo ora? [s/N]: " choice
  if [[ "$choice" =~ ^[Ss]$ ]]; then
    echo "🚀 Avvio del container ros2-dev..."
    ssh "$RPI" "cd $DOCKER_TARGET_DIR && ./start_ros2_container.sh"
    sleep 3
  else
    echo "⏭️ Container lasciato spento."
  fi
else
  echo "✅ Container ros2-dev attivo."
  read -p "🔄 Vuoi riavviarlo per applicare le modifiche? [s/N]: " restart_choice
  if [[ "$restart_choice" =~ ^[Ss]$ ]]; then
    echo "🛑 Arresto del container ros2-dev..."
    ssh "$RPI" "cd $DOCKER_TARGET_DIR && ./stop_ros2_container.sh"
    sleep 2
    echo "🚀 Riavvio del container ros2-dev..."
    ssh "$RPI" "cd $DOCKER_TARGET_DIR && ./start_ros2_container.sh"
    sleep 3
  else
    echo "⏭️ Container lasciato in esecuzione senza riavvio."
  fi
fi


# ============================================================
# 🔁 Mirror pacchetti ROS 2 (RPi5 = copia perfetta del laptop)
# ============================================================
echo "📤 Mirror pacchetti ROS 2 verso $TARGET_DIR..."

for pkg in "${PACKAGES[@]}"; do
  if [[ -d "$SRC_DIR/$pkg" ]]; then
    echo "   → $pkg"
    rsync -az --delete "$SRC_DIR/$pkg/" "$RPI:$TARGET_DIR/$pkg/"
  else
    echo "⚠️ Pacchetto mancante: $pkg (skippato)"
  fi
done

echo "✅ Sincronizzazione completata (RPi5 = copia perfetta del laptop)."

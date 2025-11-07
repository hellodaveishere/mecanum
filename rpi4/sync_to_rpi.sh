#!/bin/bash

# ============================================================
# 🔄 Sincronizza progetto ROS 2 dal laptop al Raspberry Pi 4
# ============================================================

# 📘 ISTRUZIONI D'USO:
# 1. Assicurati che il Raspberry Pi 4 sia acceso e connesso alla rete.
# 2. Verifica che SSH sia attivo sul Raspberry Pi.
# 3. Configura i parametri RPI_USER, RPI_HOST e RPI_TARGET_DIR qui sotto.
# 4. Esegui questo script dal laptop:
#    ./sync_to_rpi.sh
#
# Questo script copierà:
# - Tutto il codice ROS 2 da ~/ros2-dev-container/src/
# - Tutti i file di configurazione da ~/ros2-dev-container/rpi4/
# Verso: /home/pi/ros2-dev-container/ sul Raspberry Pi 4

# 📁 STRUTTURA DELLE CARTELLE

# 💻 Laptop (ambiente di sviluppo)
# ~/ros2-dev-container/
# ├── src/                         # Codice ROS 2 sviluppato
# └── rpi4/                        # Configurazione per Raspberry Pi 4
#     ├── docker-compose.yaml
#     ├── Dockerfile
#     ├── start_ros2_container.sh
#     ├── stop_ros2_container.sh
#     └── sync_to_rpi.sh           # Questo script

# 🍓 Raspberry Pi 4 (ambiente di esecuzione)
# /home/pi/ros2-dev-container/
# ├── src/                         # Codice ROS 2 copiato dal laptop
# ├── docker-compose.yaml
# ├── Dockerfile
# ├── start_ros2_container.sh
# └── stop_ros2_container.sh

# 📍 CONFIGURA QUI
RPI_USER=pi
RPI_HOST=192.168.1.42
RPI_TARGET_DIR=/home/pi/ros2-dev-container

# 📁 Cartelle locali
LOCAL_SRC_DIR="$(dirname "$0")/../src"
LOCAL_RPI4_DIR="$(dirname "$0")"

echo "📤 Copia dei file verso $RPI_USER@$RPI_HOST:$RPI_TARGET_DIR ..."

# 🔁 Copia codice ROS 2
rsync -avz --delete \
  "$LOCAL_SRC_DIR/" \
  "$RPI_USER@$RPI_HOST:$RPI_TARGET_DIR/src/"

# 🔁 Copia configurazione e script
# 🔄 Sincronizzazione file con Raspberry Pi 4
# rsync -avz --delete:
# -a : modalità archivio (mantiene permessi, timestamp, simboli, ecc.)
# -v : verbose (mostra i file copiati)
# -z : comprime i dati durante il trasferimento
# --delete : elimina sul Raspberry Pi i file che non esistono più sul laptop
# ⚠️ Attenzione: --delete rimuove i file obsoleti sul dispositivo remoto!
# Questo comando garantisce che la cartella di destinazione sia una copia esatta di quella locale.
rsync -avz --delete \
  "$LOCAL_RPI4_DIR/docker-compose.yaml" \
  "$LOCAL_RPI4_DIR/Dockerfile" \
  "$LOCAL_RPI4_DIR/start_ros2_container.sh" \
  "$LOCAL_RPI4_DIR/stop_ros2_container.sh" \
  "$RPI_USER@$RPI_HOST:$RPI_TARGET_DIR/"

echo "✅ Copia completata!"
echo "👉 ssh $RPI_USER@$RPI_HOST && cd ros2-dev-container && ./start_ros2_container.sh"

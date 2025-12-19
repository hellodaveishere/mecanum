#!/bin/bash

# ============================================================
# 🔄 Sincronizza progetto ROS 2 dal laptop al Raspberry Pi 4
# ============================================================

# 📘 ISTRUZIONI D'USO:
# 1. Assicurati che il Raspberry Pi 4 sia acceso e connesso alla rete.
# 2. Verifica che SSH sia attivo sul Raspberry Pi.
# 3. Esegui questo script dal laptop:
#    ./sync_to_rpi.sh
#
# Questo script copierà:
# - I pacchetti ROS 2 mecanum_base/ e sllider_ros2/ da ~/ros2-dev-container/src/
# - I file docker-compose.yaml, Dockerfile, start_ros2_container.sh e stop_ros2_container.sh da ~/ros2-dev-container/rpi4/
# - Verso: /home/ws/src/ e /home/pi/ros2-dev-container/ sul Raspberry Pi 4
# - Che è montato come volume nel container ros2-dev
#
# 📁 STRUTTURA DELLE CARTELLE

# 💻 Laptop (ambiente di sviluppo)
# ~/ros2-dev-container/
# ├── src/                         # Codice ROS 2 sviluppato
# │   ├── mecanum_base/
# │   └── sllider_ros2/
# └── rpi4/                        # Configurazione per Raspberry Pi 4
#     ├── docker-compose.yaml
#     ├── Dockerfile
#     ├── start_ros2_container.sh
#     ├── stop_ros2_container.sh
#     └── sync_to_rpi.sh           # Questo script

# 🍓 Raspberry Pi 4 (ambiente di esecuzione)
# IP: 192.168.1.42
# /home/ws/src/                    # Volume montato nel container ros2-dev
# ├── mecanum_base/
# └── sllider_ros2/
# /home/pi/ros2-dev-container/     # Contiene i file di configurazione Docker
# ============================================================

# 📍 Parametri fissi
RPI_USER="pi"
RPI_HOST="192.168.188.41"
RPI="$RPI_USER@$RPI_HOST"

SRC_DIR="../src"
RPI4_DIR="."
TARGET_DIR="/home/pi/ws/src"
DOCKER_TARGET_DIR="/home/pi/ros2-dev-container"

PACKAGES=("mecanum_base" "sllidar_ros2")
DOCKER_FILES=("docker-compose.yaml" "Dockerfile" "start_ros2_container.sh" "stop_ros2_container.sh")

# 📦 Copia dei file Docker e degli script di gestione container
# Questi file sono necessari per costruire e controllare il container ros2-dev sul Raspberry Pi
for file in "${DOCKER_FILES[@]}"; do
  if [[ -f "$RPI4_DIR/$file" ]]; then
    echo "📤 Copia del file: $file"
    rsync -avz "$RPI4_DIR/$file" "$RPI:$DOCKER_TARGET_DIR/"
  else
    echo "⚠️ Attenzione: il file $file non esiste in $RPI4_DIR, salto..."
  fi
done

# 🔍 Verifica se il container ros2-dev è attivo sul Raspberry Pi
echo "🔎 Controllo stato del container ros2-dev su $RPI_HOST..."
CONTAINER_STATUS=$(ssh "$RPI" docker ps --filter "name=ros2-dev" --filter "status=running" --format "{{.Names}}")

if [[ "$CONTAINER_STATUS" != "ros2-dev" ]]; then
  echo "⚠️ Il container ros2-dev non è attivo su $RPI_HOST."
  read -p "👉 Vuoi avviarlo ora? [s/N]: " choice
  if [[ "$choice" =~ ^[Ss]$ ]]; then
    echo "🚀 Avvio del container ros2-dev..."
    ssh "$RPI" 'cd ~/ros2-dev-container && ./start_ros2_container.sh'
    sleep 3
  else
    echo "✅ Il container ros2-dev non è attivo."
  fi
else
  echo "✅ Il container ros2-dev è attivo."

  # ⚠️ I file Docker e gli script sono stati aggiornati, ma il container è già in esecuzione.
  # Per applicare le modifiche, è necessario riavviarlo.
  read -p "🔄 Vuoi riavviare il container per applicare le modifiche? [s/N]: " restart_choice
  if [[ "$restart_choice" =~ ^[Ss]$ ]]; then
    echo "🛑 Arresto del container ros2-dev..."
    ssh "$RPI" 'cd ~/ros2-dev-container && ./stop_ros2_container.sh'
    sleep 2
    echo "🚀 Riavvio del container ros2-dev..."
    ssh "$RPI" 'cd ~/ros2-dev-container && ./start_ros2_container.sh'
    sleep 3
  else
    echo "⏭️ Il container continuerà a funzionare con la configurazione precedente."
  fi
fi

# 🔁 Sincronizzazione dei pacchetti ROS 2
for pkg in "${PACKAGES[@]}"; do
  if [[ -d "$SRC_DIR/$pkg" ]]; then
    echo "📤 Copia del pacchetto: $pkg"

    # rsync -avz --delete:
    # -a : modalità archivio (mantiene permessi, timestamp, simboli, ecc.)
    # -v : verbose (mostra i file copiati)
    # -z : comprime i dati durante il trasferimento
    # --delete : elimina sul Raspberry Pi i file che non esistono più sul laptop
    # ⚠️ Attenzione: --delete rimuove i file obsoleti sul dispositivo remoto!
    # Questo comando garantisce che la cartella di destinazione sia una copia esatta di quella locale.
    rsync -avz --delete "$SRC_DIR/$pkg/" "$RPI:$TARGET_DIR/$pkg/"
  else
    echo "⚠️ Attenzione: la cartella $pkg non esiste in $SRC_DIR, salto..."
  fi
done

echo "✅ Sincronizzazione completata."

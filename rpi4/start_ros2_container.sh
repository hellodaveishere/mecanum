#!/bin/bash

# ============================================================
# 🔄 Sincronizza progetto ROS 2 dal laptop al Raspberry Pi 4
# ============================================================

# 📘 ISTRUZIONI D'USO:
# 1. Assicurati che il Raspberry Pi 4 sia acceso e connesso alla rete.
# 2. Verifica che SSH sia attivo sul Raspberry Pi.
# 3. Esegui questo script dal laptop:
#    ./sync_to_rpi.sh [--force]
#
# Questo script copierà:
# - I pacchetti ROS 2 mecanum_base/ e sllider_ros2/ da ~/ros2-dev-container/src/
# - Verso: /home/ws/src/ sul Raspberry Pi 4
# - Che è montato come volume nel container ros2-dev
# - Se il container è attivo, chiederà se vuoi riavviarlo per applicare le modifiche (o lo farà automaticamente con --force)
# - Se il container è spento, chiederà se vuoi avviarlo (o lo farà automaticamente con --force)

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
# ============================================================

# 📍 Parametri fissi
RPI_USER="pi"

# 🧠 IP del Raspberry Pi 4 sulla rete locale (non è l’IP del container!)
# Il container ros2-dev usa network_mode: host, quindi condivide l’IP del Raspberry Pi.
# Questo IP deve essere statico o riservato nel router per evitare cambiamenti.
RPI_HOST="192.168.1.42"

RPI="$RPI_USER@$RPI_HOST"
SRC_DIR="./src"
TARGET_DIR="/home/ws/src"
PACKAGES=("mecanum_base" "sllider_ros2")

# ⚙️ Modalità forzata
FORCE_MODE=false
if [[ "$1" == "--force" ]]; then
  FORCE_MODE=true
  echo "⚡ Modalità forzata attiva: nessuna conferma richiesta"
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

# 🔍 Verifica stato del container ros2-dev
echo "🔎 Controllo stato del container ros2-dev su $RPI_HOST..."
CONTAINER_STATUS=$(ssh "$RPI" docker ps --filter "name=ros2-dev" --filter "status=running" --format "{{.Names}}")

if [[ "$CONTAINER_STATUS" == "ros2-dev" ]]; then
  echo "ℹ️ Il container ros2-dev è attualmente attivo."
  if $FORCE_MODE; then
    echo "🔄 Riavvio automatico del container ros2-dev..."
    ssh "$RPI" 'cd ~/ros2-dev-container && ./stop_ros2_container.sh && ./start_ros2_container.sh'
  else
    read -p "🔄 Vuoi riavviarlo per applicare le modifiche? [s/N]: " choice
    if [[ "$choice" =~ ^[Ss]$ ]]; then
      echo "🛑 Arresto e riavvio del container ros2-dev..."
      ssh "$RPI" 'cd ~/ros2-dev-container && ./stop_ros2_container.sh && ./start_ros2_container.sh'
    else
      echo "⏭️ Riavvio annullato. Il container continuerà a girare con i file aggiornati."
    fi
  fi
else
  echo "⚠️ Il container ros2-dev non è attivo."
  if $FORCE_MODE; then
    echo "🚀 Avvio automatico del container ros2-dev..."
    ssh "$RPI" 'cd ~/ros2-dev-container && ./start_ros2_container.sh'
  else
    read -p "🚀 Vuoi avviarlo ora? [s/N]: " choice
    if [[ "$choice" =~ ^[Ss]$ ]]; then
      echo "🔧 Avvio del container ros2-dev..."
      ssh "$RPI" 'cd ~/ros2-dev-container && ./start_ros2_container.sh'
    else
      echo "⏭️ Avvio annullato. Il container rimane spento."
    fi
  fi
fi

echo "✅ Sincronizzazione completata."

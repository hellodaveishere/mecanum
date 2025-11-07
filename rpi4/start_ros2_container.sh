#!/bin/bash

# ============================================================
# 🚀 Avvio container ROS 2 con o senza GUI, build solo se necessario
# ============================================================
# 📘 USO:
#   ./start_ros2_container.sh        # Avvio senza GUI
#   ./start_ros2_container.sh --gui # Avvio con GUI (es. RViz, rqt)
#
# 📦 REQUISITI:
# - Docker e Docker Compose installati
# - X11 attivo se si usa GUI
# - DISPLAY impostato correttamente sul sistema host
# - File .dockerignore configurato per escludere cache inutili
# ============================================================

# 🔧 Impostazione profilo Docker Compose
PROFILE="default"
BUILD_FLAG=""
HASH_FILE=".last_build.hash"

# 🖼️ Gestione GUI
if [[ "$1" == "--gui" ]]; then
  export DISPLAY=:0
  PROFILE="gui"
  echo "✅ GUI abilitata"
else
  unset DISPLAY
  echo "🚫 GUI disabilitata (default)"
fi

# 🔍 Calcolo checksum dei file rilevanti per la build
# Include: Dockerfile, docker-compose.yaml, e tutti i file in src/
CURRENT_HASH=$(find Dockerfile docker-compose.yaml src/ -type f -exec sha256sum {} + | sort | sha256sum | cut -d ' ' -f1)

# 📦 Verifica se il checksum è cambiato rispetto all’ultima build
if [[ ! -f "$HASH_FILE" || "$CURRENT_HASH" != "$(cat $HASH_FILE)" ]]; then
  echo "🔁 Modifiche rilevate: ricostruzione dell'immagine..."
  BUILD_FLAG="--build"
  echo "$CURRENT_HASH" > "$HASH_FILE"
else
  echo "✅ Nessuna modifica: uso immagine esistente"
fi

# 🐳 Avvio del container con profilo e build condizionata
echo "🔧 Avvio container ROS 2 con profilo: $PROFILE ..."
docker compose --profile "$PROFILE" up $BUILD_FLAG -d

# 🧠 Istruzioni post-avvio
echo ""
echo "⏳ Container avviato."
echo "👉 Per accedere alla shell ROS 2:"
echo "   docker exec -it ros2-dev bash"

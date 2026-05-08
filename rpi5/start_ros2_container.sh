#!/bin/bash

set -euo pipefail

# ============================================================
# Configurazione utente per accesso al container
# ------------------------------------------------------------
# CONTAINER_USER può essere definito:
#   - via variabile ambiente:  CONTAINER_USER=robot ./run.sh
#   - via CLI:                 --user=robot
# Default: ws (coerente con workspace in /home/ws)
# ============================================================
CONTAINER_USER="${CONTAINER_USER:-ws}"

# Default
USE_GUI=false
OPEN_SHELL=true
RUN_LAUNCH=false

print_help() {
  cat <<EOF
Uso: $0 [opzioni]

Opzioni:
  --gui             Usa il container con supporto GUI
  --headless        Usa il container headless

  --shell           Entra in una shell interattiva nel container (default)
  --no-shell        Non aprire la shell

  --launch          Esegui ros2 launch nel container (esclude --shell)

  --user=<nome>     Specifica l'utente con cui entrare nel container
                    (default: ws, override tramite variabile ambiente CONTAINER_USER)

  -h, --help        Mostra questo messaggio
EOF
}

# --- Parse args ---
for arg in "$@"; do
  case $arg in
    --gui) USE_GUI=true ;;
    --headless) USE_GUI=false ;;
    --shell) OPEN_SHELL=true ;;
    --no-shell) OPEN_SHELL=false ;;
    --launch) RUN_LAUNCH=true ;;
    --user=*)
      CONTAINER_USER="${arg#*=}"
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    *)
      echo "Opzione sconosciuta: $arg"
      exit 1
      ;;
  esac
done

# launch → niente shell
if $RUN_LAUNCH; then
  OPEN_SHELL=false
fi

# ============================================================
# Controllo modifiche a Dockerfile / docker-compose.yml
# ============================================================

BUILD_STAMP=".last_build"
NEED_REBUILD=false

if [[ ! -f "$BUILD_STAMP" ]]; then
    NEED_REBUILD=true
else
    if [[ Dockerfile -nt "$BUILD_STAMP" ]]; then
        echo "⚠️  Dockerfile è stato modificato dopo l'ultimo build."
        NEED_REBUILD=true
    fi
    if [[ docker-compose.yml -nt "$BUILD_STAMP" ]]; then
        echo "⚠️  docker-compose.yml è stato modificato dopo l'ultimo build."
        NEED_REBUILD=true
    fi
fi

if $NEED_REBUILD; then
    echo "🔄 I file di build sono cambiati. Vuoi ricostruire il container?"
    read -p "👉 [s/N]: " REPLY
    if [[ "$REPLY" =~ ^[sS]$ ]]; then
        echo "🔧 Ricostruzione del container..."
        docker compose build
        date > "$BUILD_STAMP"
        echo "✅ Build completata."
    else
        echo "⏭️  Build saltata su richiesta dell'utente."
    fi
fi

# ============================================================
# Selezione container (GUI / headless)
# ============================================================

if $USE_GUI; then
  CONTAINER_NAME="ros2-dev-gui"
  PROFILE="--profile gui"
  echo "▶️ Modalità GUI: container $CONTAINER_NAME"
else
  CONTAINER_NAME="ros2-dev"
  PROFILE=""
  echo "▶️ Modalità headless: container $CONTAINER_NAME"
fi

# ============================================================
# Avvio container (idempotente)
# ============================================================

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "♻️  Container $CONTAINER_NAME esistente: lo avvio/riuso..."
  docker compose $PROFILE up -d
else
  echo "🧱 Container $CONTAINER_NAME non esiste: build + up..."
  docker compose $PROFILE build
  docker compose $PROFILE up -d
fi

sleep 1

# ============================================================
# Controllo esistenza utente nel container + fallback a root
# ============================================================

USER_EXISTS=false
HOME_EXISTS=false

if docker exec "$CONTAINER_NAME" id "$CONTAINER_USER" >/dev/null 2>&1; then
  USER_EXISTS=true
fi

if docker exec "$CONTAINER_NAME" test -d "/home/$CONTAINER_USER"; then
  HOME_EXISTS=true
fi

if $USER_EXISTS && $HOME_EXISTS; then
  EFFECTIVE_USER="$CONTAINER_USER"
else
  echo "⚠️  L'utente '$CONTAINER_USER' non è valido nel container."
  if ! $USER_EXISTS; then
    echo "   - Utente inesistente"
  fi
  if ! $HOME_EXISTS; then
    echo "   - Home directory /home/$CONTAINER_USER mancante"
  fi
  echo "➡️  Fallback automatico a utente 'root'."
  EFFECTIVE_USER="root"
fi

# ============================================================
# Shell interattiva
# ============================================================

if $OPEN_SHELL; then
  echo "🔧 Entrando nel container $CONTAINER_NAME come utente '$EFFECTIVE_USER'..."
  docker exec -it -u "$EFFECTIVE_USER" "$CONTAINER_NAME" bash --login -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /home/ws/install/setup.bash && \
     exec bash --login"
  exit 0
fi

# ============================================================
# Launch ROS2
# ============================================================

if $RUN_LAUNCH; then
  echo "🚀 Avvio ROS 2 launch nel container $CONTAINER_NAME come utente '$EFFECTIVE_USER'..."
  docker exec -it -u "$EFFECTIVE_USER" "$CONTAINER_NAME" bash --login -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /home/ws/install/setup.bash && \
     ros2 launch mecanum_base bringup.launch.py"
  exit 0
fi

echo "✅ Container $CONTAINER_NAME avviato/riusato."

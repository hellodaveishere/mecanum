#!/bin/bash

set -euo pipefail

# Default
USE_GUI=true
OPEN_SHELL=true
RUN_LAUNCH=false

print_help() {
  cat <<EOF
Uso: $0 [opzioni]

Opzioni:
  --gui         Usa il container con supporto GUI (default)
  --headless    Usa il container headless

  --shell       Entra in una shell interattiva nel container (default)
  --no-shell    Non aprire la shell

  --launch      Esegui ros2 launch nel container (esclude --shell)

  -h, --help    Mostra questo messaggio

Combinazioni:
  (nessuna opzione)        GUI + shell interattiva
  --headless               Headless + shell
  --gui --no-shell         Avvia/riusa container GUI
  --headless --no-shell    Avvia/riusa container headless
  --gui --launch           Avvia/riusa GUI e lancia ros2 launch
  --headless --launch      Avvia/riusa headless e lancia ros2 launch

In tutti i casi il container viene riusato se esiste già.
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

# --- Container selection ---
if $USE_GUI; then
  CONTAINER_NAME="ros2-dev-gui"
  PROFILE="--profile gui"
  echo "▶️ Modalità GUI: container $CONTAINER_NAME"
else
  CONTAINER_NAME="ros2-dev"
  PROFILE=""
  echo "▶️ Modalità headless: container $CONTAINER_NAME"
fi

# --- Ensure container exists and is running ---
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "♻️  Container $CONTAINER_NAME esistente: lo avvio/riuso..."
  docker compose $PROFILE up -d
else
  echo "🧱 Container $CONTAINER_NAME non esiste: build + up..."
  docker compose $PROFILE build
  docker compose $PROFILE up -d
fi

sleep 1

# --- Shell interattiva con ROS già sorgato ---
if $OPEN_SHELL; then
  echo "🔧 Entrando nel container $CONTAINER_NAME con ambiente ROS 2..."
  docker exec -it "$CONTAINER_NAME" bash --login -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /home/ws/install/setup.bash && \
     exec bash --login"
  exit 0
fi

# --- Launch ROS 2 ---
if $RUN_LAUNCH; then
  echo "🚀 Avvio ROS 2 launch nel container $CONTAINER_NAME ..."
  docker exec -it "$CONTAINER_NAME" bash --login -c \
    "source /opt/ros/jazzy/setup.bash && \
     source /home/ws/install/setup.bash && \
     ros2 launch mecanum_base bringup.launch.py"
  exit 0
fi

echo "✅ Container $CONTAINER_NAME avviato/riusato."
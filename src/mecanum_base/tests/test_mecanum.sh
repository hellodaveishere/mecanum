#!/bin/bash
# ============================================
# Script di test per Mecanum Velocity Controller
# Invia comandi di velocità alle 4 ruote
# nell'ordine definito in controllers.yaml:
#   [front_left, front_right, rear_left, rear_right]
# ============================================

set -euo pipefail

TOPIC="/mecanum_velocity_controller/commands"
MSG_TYPE="std_msgs/msg/Float64MultiArray"
RATE=5   # Hz

send_cmd() {
    local duration="$1"; shift
    local values=("$@")

    # Costruisci lista separata da virgole per YAML
    local data_csv
    data_csv="$(IFS=,; echo "${values[*]}")"

    # Calcola quante pubblicazioni fare
    local times=$(( duration * RATE ))

    echo -e "\n\033[1;34m>>> Movimento: ${values[*]} per ${duration}s\033[0m"
    echo "    [FL, FR, RL, RR] in rad/s"
    echo "    Pubblico su: $TOPIC @ ${RATE} Hz per ${times} messaggi"

    # Pubblica un numero finito di messaggi e termina da solo (niente kill)
    ros2 topic pub -r "${RATE}" --times "${times}" "$TOPIC" "$MSG_TYPE" -- \
"{data: [${data_csv}]}"
    # Piccola pausa per consentire una chiusura pulita del processo precedente
    sleep 0.2
}

echo -e "\033[1;32m=== Avvio test Mecanum Velocity Controller ===\033[0m"

# Avanti
send_cmd 3 1.0 1.0 1.0 1.0

# Indietro
send_cmd 3 -1.0 -1.0 -1.0 -1.0

# Rotazione oraria
send_cmd 3 1.0 -1.0 1.0 -1.0

# Rotazione antioraria
send_cmd 3 -1.0 1.0 -1.0 1.0

# Traslazione laterale destra
send_cmd 3 1.0 -1.0 -1.0 1.0

# Traslazione laterale sinistra
send_cmd 3 -1.0 1.0 1.0 -1.0

# Fermo
send_cmd 3 0.0 0.0 0.0 0.0

echo -e "\033[1;32m=== Test completato ===\033[0m"

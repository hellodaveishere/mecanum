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
RATE=50   # Hz

send_cmd() {
    local duration="$1"; shift
    local values=("$@")

    local data_csv
    data_csv="$(IFS=,; echo "${values[*]}")"

    local times=$(( duration * RATE ))

    echo -e "\n\033[1;34m>>> Movimento: ${values[*]} per ${duration}s\033[0m"
    echo "    [FL, FR, RL, RR] in rad/s"
    echo "    Pubblico su: $TOPIC @ ${RATE} Hz per ${times} messaggi"

    ros2 topic pub -r "${RATE}" --times "${times}" "$TOPIC" "$MSG_TYPE" -- \
"{data: [${data_csv}]}"
    sleep 0.2
}

# ============================================
# NUOVA FUNZIONE: TEST MOTORE SINGOLO
# ============================================
test_single_motor() {
    echo -e "\n\033[1;33m=== Test motore singolo ===\033[0m"
    echo "Scegli il motore da testare:"
    echo "  1) Front Left"
    echo "  2) Front Right"
    echo "  3) Rear Left"
    echo "  4) Rear Right"
    read -rp "Motore (1-4): " motor

    read -rp "Velocità (rad/s, es: 1.0 o -1.0): " speed
    read -rp "Durata (s): " duration

    local fl=0 fr=0 rl=0 rr=0

    case "$motor" in
        1) fl="$speed" ;;
        2) fr="$speed" ;;
        3) rl="$speed" ;;
        4) rr="$speed" ;;
        *) echo "Motore non valido"; return ;;
    esac

    echo -e "\033[1;34m>>> Test motore $motor con velocità $speed rad/s per $duration s\033[0m"
    send_cmd "$duration" "$fl" "$fr" "$rl" "$rr"

    # 🔥 STOP AUTOMATICO DOPO IL TEST
    echo -e "\033[1;31m>>> STOP motore $motor\033[0m"
    send_cmd 1 0 0 0 0
}

echo -e "\033[1;32m=== Avvio test Mecanum Velocity Controller ===\033[0m"

# ============================================
# MENU PRINCIPALE
# ============================================
while true; do
    echo -e "\n\033[1;36mScegli un test:\033[0m"
    echo "  1) Test completo (movimenti base)"
    echo "  2) Test motore singolo"
    echo "  0) Esci"
    read -rp "> " choice

    case "$choice" in
        1)
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
            ;;
        2)
            test_single_motor
            ;;
        0)
            echo -e "\033[1;32m=== Test completato ===\033[0m"
            exit 0
            ;;
        *)
            echo "Scelta non valida."
            ;;
    esac
done

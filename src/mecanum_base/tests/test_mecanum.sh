#!/bin/bash

set -uo pipefail

# Inizializza ROS2 (adatta se usi un altro path)
if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Controllo bc (anche se ora non lo usiamo più direttamente, ma può servirti)
if ! command -v bc >/dev/null 2>&1; then
    echo "Errore: installa bc con: sudo apt install bc"
    exit 1
fi

TOPIC="/mecanum_velocity_controller/commands"
MSG_TYPE="std_msgs/msg/Float64MultiArray"
RATE=50

motor_sel="none"

send_cmd() {
    local duration="$1"; shift
    local values=("$@")
    local data_csv
    data_csv="$(IFS=,; echo "${values[*]}")"
    local times=$(( duration * RATE ))

    echo -e "\n>>> Movimento: ${values[*]} per ${duration}s"

    # ROS 2 Jazzy: silenzia SOLO stdout, NON stderr
    ros2 topic pub -r "${RATE}" --times "${times}" "$TOPIC" "$MSG_TYPE" -- \
        "{data: [${data_csv}]}" > /dev/null

    sleep 0.2
}

read_wheel_speeds() {
    local fl="$1"
    local fr="$2"
    local rl="$3"
    local rr="$4"

    echo -e "\n>>> Misurazione velocità media..."

    # Piccolo delay per dare tempo a ros2 topic pub di iniziare a pubblicare
    sleep 0.3

    python3 wheel_monitor.py \
        --duration "$TEST_DURATION" \
        --target "$fl" "$fr" "$rl" "$rr" \
        --motor "$motor_sel"
}

test_single_motor() {
    echo -e "\n=== Test motore singolo ==="
    echo "  1) Front Left"
    echo "  2) Front Right"
    echo "  3) Rear Left"
    echo "  4) Rear Right"
    read -rp "Motore (1-4): " motor_sel

    read -rp "Velocità (rad/s): " speed
    read -rp "Durata (s): " TEST_DURATION

    local fl=0 fr=0 rl=0 rr=0

    case "$motor_sel" in
        1) fl="$speed" ;;
        2) fr="$speed" ;;
        3) rl="$speed" ;;
        4) rr="$speed" ;;
        *) echo "Motore non valido"; return ;;
    esac

    send_cmd "$TEST_DURATION" "$fl" "$fr" "$rl" "$rr"
    read_wheel_speeds "$fl" "$fr" "$rl" "$rr"

    echo -e "\n>>> STOP motore"
    send_cmd 1 0 0 0 0
}

echo -e "=== Avvio test Mecanum Velocity Controller ==="

while true; do
    echo -e "\nScegli un test:"
    echo "  1) Test completo (solo comandi, senza misura)"
    echo "  2) Test motore singolo (con misura)"
    echo "  0) Esci"
    read -rp "> " choice

    case "$choice" in
        1)
            send_cmd 3 1 1 1 1
            send_cmd 3 -1 -1 -1 -1
            send_cmd 3 1 -1 1 -1
            send_cmd 3 -1 1 -1 1
            send_cmd 3 1 -1 -1 1
            send_cmd 3 -1 1 1 -1
            send_cmd 3 0 0 0 0
            ;;
        2)
            test_single_motor
            ;;
        0)
            echo "Uscita."
            exit 0
            ;;
        *)
            echo "Scelta non valida."
            ;;
    esac
done

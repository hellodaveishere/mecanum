#!/bin/bash
# ============================================
# Script di test per Mecanum Velocity Controller
# ============================================

set -uo pipefail

# 🔥 INIZIALIZZA AMBIENTE ROS (ADATTA ALLA TUA INSTALLAZIONE)
if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f /opt/ros/foxy/setup.bash ]; then
        source /opt/ros/foxy/setup.bash
    elif [ -f /opt/ros/galactic/setup.bash ]; then
        source /opt/ros/galactic/setup.bash
    fi
fi

TOPIC="/mecanum_velocity_controller/commands"
MSG_TYPE="std_msgs/msg/Float64MultiArray"
FEEDBACK_TOPIC="/joint_states"
RATE=50
CSV_FILE="mecanum_test_log.csv"

OUTPUT_MODE=1
motor_sel="none"

init_csv() {
    if [[ ! -f "$CSV_FILE" ]]; then
        echo "timestamp,motor,target_FL,target_FR,target_RL,target_RR,meas_FL,meas_FR,meas_RL,meas_RR,err_FL,err_FR,err_RL,err_RR" > "$CSV_FILE"
    fi
}

send_cmd() {
    local duration="$1"; shift
    local values=("$@")
    local data_csv
    data_csv="$(IFS=,; echo "${values[*]}")"
    local times=$(( duration * RATE ))

    echo -e "\n\033[1;34m>>> Movimento: ${values[*]} per ${duration}s\033[0m"
    ros2 topic pub -r "${RATE}" --times "${times}" "$TOPIC" "$MSG_TYPE" -- "{data: [${data_csv}]}"
    sleep 0.2
}

read_wheel_speeds() {
    local target=("$@")

    echo -e "\033[1;35m>>> Lettura velocità ruote da /joint_states...\033[0m"

    local msg=""
    for attempt in {1..10}; do
        echo "Tentativo lettura /joint_states ($attempt/10)..."
        msg=$(ros2 topic echo "$FEEDBACK_TOPIC" --once 2>/dev/null || true)
        if [[ -n "$msg" ]]; then break; fi
        sleep 0.2
    done

    if [[ -z "$msg" ]]; then
        echo "❌ Impossibile leggere /joint_states dopo 10 tentativi."
        return 0
    fi

    local names velocities
    names=$(echo "$msg" | sed -n '/name:/,/position:/p' | sed '1d;$d' | sed "s/'//g" | sed 's/- //g' | sed 's/ //g' | sed '/^$/d')
    velocities=$(echo "$msg" | sed -n '/velocity:/,/effort:/p' | sed '1d;$d' | sed 's/- //g' | sed 's/ //g' | sed '/^$/d')

    mapfile -t name_arr <<< "$names"
    mapfile -t vel_arr <<< "$velocities"

    declare -A idx
    for i in "${!name_arr[@]}"; do
        case "${name_arr[$i]}" in
            front_left_wheel_joint)  idx[FL]=$i ;;
            front_right_wheel_joint) idx[FR]=$i ;;
            rear_left_wheel_joint)   idx[RL]=$i ;;
            rear_right_wheel_joint)  idx[RR]=$i ;;
        esac
    done

    local fl="nan" fr="nan" rl="nan" rr="nan"
    [[ -n "${idx[FL]:-}" ]] && fl="${vel_arr[${idx[FL]}]:-nan}"
    [[ -n "${idx[FR]:-}" ]] && fr="${vel_arr[${idx[FR]}]:-nan}"
    [[ -n "${idx[RL]:-}" ]] && rl="${vel_arr[${idx[RL]}]:-nan}"
    [[ -n "${idx[RR]:-}" ]] && rr="${vel_arr[${idx[RR]}]:-nan}"

    local measured=( "$fl" "$fr" "$rl" "$rr" )
    local err=( "nan" "nan" "nan" "nan" )

    if [[ "$OUTPUT_MODE" == "1" || "$OUTPUT_MODE" == "3" ]]; then
        echo -e "\n\033[1;36mVelocità misurate (rad/s):\033[0m"
        printf "  FL: %s\n" "$fl"
        printf "  FR: %s\n" "$fr"
        printf "  RL: %s\n" "$rl"
        printf "  RR: %s\n" "$rr"
    fi

    for i in {0..3}; do
        local t="${target[$i]}"
        local m="${measured[$i]}"
        if echo "$t" | grep -qE '^0(\.0+)?$'; then
            err[$i]="nan"
        elif [[ "$m" == "nan" || "$m" == ".nan" ]]; then
            err[$i]="nan"
        else
            err[$i]=$(echo "scale=2; (($m - $t) / $t) * 100" | bc -l)
        fi
    done

    if [[ "$OUTPUT_MODE" == "1" || "$OUTPUT_MODE" == "3" ]]; then
        echo -e "\n\033[1;36mErrore percentuale:\033[0m"
        printf "  FL: %s %%\n" "${err[0]}"
        printf "  FR: %s %%\n" "${err[1]}"
        printf "  RL: %s %%\n" "${err[2]}"
        printf "  RR: %s %%\n" "${err[3]}"
    fi

    if [[ "$OUTPUT_MODE" == "2" || "$OUTPUT_MODE" == "3" ]]; then
        init_csv
        local timestamp
        timestamp=$(date +"%Y-%m-%d %H:%M:%S")
        echo "$timestamp,$motor_sel,${target[0]},${target[1]},${target[2]},${target[3]},$fl,$fr,$rl,$rr,${err[0]},${err[1]},${err[2]},${err[3]}" >> "$CSV_FILE"
        echo -e "\033[1;32mRisultati salvati in $CSV_FILE\033[0m"
    fi
}

test_single_motor() {
    echo -e "\n\033[1;33m=== Test motore singolo ===\033[0m"
    echo "  1) Front Left"
    echo "  2) Front Right"
    echo "  3) Rear Left"
    echo "  4) Rear Right"
    read -rp "Motore (1-4): " motor_sel

    read -rp "Velocità (rad/s): " speed
    read -rp "Durata (s): " duration

    local fl=0 fr=0 rl=0 rr=0

    case "$motor_sel" in
        1) fl="$speed" ;;
        2) fr="$speed" ;;
        3) rl="$speed" ;;
        4) rr="$speed" ;;
        *) echo "Motore non valido"; return ;;
    esac

    send_cmd "$duration" "$fl" "$fr" "$rl" "$rr"
    read_wheel_speeds "$fl" "$fr" "$rl" "$rr"

    echo -e "\033[1;31m>>> STOP motore\033[0m"
    send_cmd 1 0 0 0 0
}

echo -e "\033[1;32m=== Avvio test Mecanum Velocity Controller ===\033[0m"

while true; do
    echo -e "\n\033[1;36mScegli un test:\033[0m"
    echo "  1) Test completo"
    echo "  2) Test motore singolo"
    echo "  3) Imposta output (console/csv)"
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
        3)
            echo "Scegli modalità output:"
            echo "  1) Solo console"
            echo "  2) Solo CSV"
            echo "  3) Console + CSV"
            read -rp "> " OUTPUT_MODE
            echo "Modalità impostata su $OUTPUT_MODE."
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

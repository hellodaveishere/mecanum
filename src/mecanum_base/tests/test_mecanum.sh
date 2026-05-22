#!/bin/bash
set -uo pipefail

trap 'echo -e "\n>>> CTRL-C rilevato: STOP motori"; send_cmd 1 0 0 0 0; exit 1' INT

# Inizializza ROS2 (adatta se usi un altro path)
if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Controllo bc (anche se ora non lo usiamo più direttamente, ma può servirti)
#if ! command -v bc >/dev/null 2>&1; then
#    echo "Errore: installa bc con: sudo apt install bc"
#    exit 1
#fi

TOPIC="/mecanum_velocity_controller/commands"
MSG_TYPE="std_msgs/msg/Float64MultiArray"
RATE=50

motor_sel="none"

send_cmd() {
    local duration="$1"; shift
    local values=("$@")

    # Converte array → CSV
    local data_csv
    data_csv="$(IFS=,; echo "${values[*]}")"

    # Numero di messaggi da pubblicare
    local times=$(( duration * RATE ))

    echo -e "\n>>> Movimento: ${values[*]} per ${duration}s"

    # Costruisco la stringa del comando ROS2
    local cmd="ros2 topic pub -r ${RATE} --times ${times} ${TOPIC} ${MSG_TYPE} -- \"{data: [${data_csv}]}\""

    # 🔍 STAMPA DEL COMANDO INVIATO
    echo ">>> Comando ROS2: $cmd"

    # Esecuzione del comando
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


###############################################
# 🔧 TUNING PID MANUALE AUTOMATICO (Assistente Umano)
###############################################
test_pid_manual() {

    echo -e "\n=== Tuning PID MANUALE (Assistente Umano) ==="
    echo "Il PID è sul microcontrollore: Kp/Ki/Kd vanno impostati a mano."
    echo "Lo script misura, analizza e ti dice quale Kp provare dopo."
    echo

    echo "Seleziona il motore:"
    echo "  1) Front Left"
    echo "  2) Front Right"
    echo "  3) Rear Left"
    echo "  4) Rear Right"
    read -rp "Motore (1-4): " motor_sel

    case "$motor_sel" in
        1|2|3|4) ;;
        *) echo "Motore non valido"; return ;;
    esac

    read -rp "Velocità target (rad/s) [default 5]: " speed
    speed=${speed:-5}

    ###############################################################
    # 🔥 PATCH LOW-SPEED: se |speed| < 3 rad/s usa PID speciale
    ###############################################################
    abs_speed=$(python3 - <<EOF
import sys
print(abs(float("$speed")))
EOF
)

    if python3 - <<EOF >/dev/null 2>&1
import sys
sys.exit(0 if float("$abs_speed") < 3.0 else 1)
EOF
    then
        echo -e "\n>>> Modalità LOW-SPEED attiva (|speed| < 3 rad/s)"
        echo "    - Ziegler–Nichols DISABILITATO"
        echo "    - Ricerca oscillazioni DISABILITATA"
        echo "    - Verranno suggeriti guadagni morbidi:"
        echo "        Kp = 1.0"
        echo "        Ki = 0.05"
        echo "        Kd = 0.0"
        echo

        local Kp_final="1.0"
        local Ki_final="0.05"
        local Kd_final="0.0"

        echo -e "\n=== RISULTATI PER VELOCITÀ ${speed} rad/s ==="
        echo "Metodo: LOW-SPEED"
        echo
        echo "Parametri PID suggeriti:"
        echo "  Kp = $Kp_final"
        echo "  Ki = $Ki_final"
        echo "  Kd = $Kd_final"

        # Log finale low-speed
        local LOG_FILE="pid_tuning_log.txt"
        echo "MODE=LOW-SPEED SPEED=${speed} Kp=${Kp_final} Ki=${Ki_final} Kd=${Kd_final}" >> "$LOG_FILE"

        echo -e "\n>>> Imposta questi valori sul microcontrollore."
        return
    fi

    read -rp "Durata step (s) [default 3]: " TEST_DURATION
    TEST_DURATION=${TEST_DURATION:-3}

    # Ricerca automatica
    local Kp=0.1
    local Kp_step_coarse=0.5
    local Kp_step_fine=0.1
    local Kp_step=$Kp_step_coarse

    # Parametri steady-state
    local steady_err_threshold=10.0
    local steady_ok_count=0
    local steady_ok_required=5

    local Ku_found=0
    local Tu=0
    local mode=""

    # Limiti massimi per Ki e Kd (clamp)
    local KI_MAX=5.0
    local KD_MAX=1.0

    # File di log
    local LOG_FILE="pid_tuning_log.txt"
    echo -e "\n>>> Log tuning in: $LOG_FILE"
    echo "=== NEW SESSION motor=${motor_sel} speed=${speed} ===" >> "$LOG_FILE"

    echo -e "\n>>> Ricerca automatica a ${speed} rad/s"
    echo "    Modalità assistente umano:"
    echo "    - Lo script propone Kp"
    echo "    - Tu lo imposti sul micro"
    echo "    - Premi INVIO"
    echo "    - Lo script misura e decide il prossimo passo"
    echo

    ###############################################
    # 1) RICERCA AUTOMATICA
    ###############################################
    while true; do

        ###########################################################
        # 🔥 ASSISTENTE UMANO: IMPOSTA Kp SUL MICRO
        ###########################################################
        echo -e "\n>>> Imposta ORA sul microcontrollore:"
        echo "      Kp = $Kp"
        echo "      Ki = 0"
        echo "      Kd = 0"
        echo "    (Aggiorna il micro e premi INVIO per continuare)"
        read -rp "" _

        echo -e "\n>>> Test con Kp = $Kp (step = $Kp_step)"

        local fl=0 fr=0 rl=0 rr=0
        case "$motor_sel" in
            1) fl="$speed" ;;
            2) fr="$speed" ;;
            3) rl="$speed" ;;
            4) rr="$speed" ;;
        esac

        send_cmd "$TEST_DURATION" "$fl" "$fr" "$rl" "$rr"

        local result
        result=$(python3 wheel_monitor.py \
            --duration "$TEST_DURATION" \
            --target "$fl" "$fr" "$rl" "$rr" \
            --motor "$motor_sel" \
            --analyze-oscillation)

        echo ">>> Risultato analisi: $result"

        # Log del test
        echo "TEST Kp=${Kp} speed=${speed} RESULT=${result}" >> "$LOG_FILE"

        ########################################
        # Caso 1 — Oscillazione stabile → ZN
        ########################################
        if echo "$result" | grep -q "^OSCILLATION_DETECTED "; then
            Ku_found="$Kp"
            Tu=$(echo "$result" | sed -n 's/^OSCILLATION_DETECTED PERIOD=\([^ ]*\).*/\1/p')
            mode="ZN"
            break
        fi

        ########################################
        # Caso 2 — Oscillazione instabile → passo fine (solo se Kp >= 1.0)
        ########################################
        if echo "$result" | grep -q "UNSTABLE_OSCILLATION"; then

            # step fine solo da Kp >= 1.0
            is_high_kp=$(python3 - <<EOF
Kp = float("$Kp")
print("YES" if Kp >= 1.0 else "NO")
EOF
)

            if [[ "$is_high_kp" == "YES" ]]; then
                Kp_step=$Kp_step_fine
                echo ">>> Passo fine attivato (Kp_step = $Kp_step_fine)"
            else
                echo ">>> Oscillazione instabile ignorata (Kp troppo basso per passo fine)"
            fi
        fi

        ########################################
        # Caso 3 — Nessuna oscillazione → criterio steady-state
        ########################################
        local err_pct
        err_pct=$(echo "$result" | sed -n 's/.*ERROR=\([-0-9.]*\).*/\1/p')

        if [[ -n "$err_pct" ]]; then
            local abs_err
            abs_err=$(python3 - <<EOF
e = float("$err_pct")
print(abs(e))
EOF
)

            echo ">>> Errore stazionario stimato: ${abs_err}%"

            if python3 - <<EOF >/dev/null 2>&1
import sys
sys.exit(0 if float("$abs_err") < float("$steady_err_threshold") else 1)
EOF
            then
                steady_ok_count=$((steady_ok_count + 1))
                echo ">>> Steady-state OK (${steady_ok_count}/${steady_ok_required})"
            else
                steady_ok_count=0
            fi

            if (( steady_ok_count >= steady_ok_required )); then
                mode="STEADY"
                break
            fi
        fi

        # Stop motore
        send_cmd 1 0 0 0 0

        ###########################################################
        # 🔥 CALCOLO NUOVO Kp (step grossolano → fine)
        ###########################################################
        Kp=$(python3 - <<EOF
print(float("$Kp") + float("$Kp_step"))
EOF
)

        # Limite sicurezza
        if python3 - <<EOF >/dev/null 2>&1
import sys
sys.exit(0 if float("$Kp") > 5.0 else 1)
EOF
        then
            echo ">>> ERRORE: Kp troppo alto senza oscillazioni né steady-state"
            send_cmd 1 0 0 0 0
            echo "ABORT Kp=${Kp} speed=${speed}" >> "$LOG_FILE"
            return
        fi
    done

    ###############################################
    # 2) CALCOLO PID
    ###############################################
    local Kp_final Ki_final Kd_final

    if [[ "$mode" == "ZN" ]]; then
        echo -e "\n>>> Uso Ziegler–Nichols"

        Kp_final=$(python3 - <<EOF
print(0.6 * float("$Ku_found"))
EOF
)
        Ki_final=$(python3 - <<EOF
Kp = float("$Kp_final")
Tu = float("$Tu")
Ki = 2 * Kp / Tu
KI_MAX = float("$KI_MAX")
print(KI_MAX if Ki > KI_MAX else Ki)
EOF
)
        Kd_final=$(python3 - <<EOF
Kp = float("$Kp_final")
Tu = float("$Tu")
Kd = Kp * Tu / 8.0
KD_MAX = float("$KD_MAX")
print(KD_MAX if Kd > KD_MAX else Kd)
EOF
)

    else
        echo -e "\n>>> Nessuna oscillazione → uso metodo low‑gain"
        Kp_final="$Kp"

        Ki_final=$(python3 - <<EOF
Kp = float("$Kp_final")
Ki = Kp / 5.0
KI_MAX = float("$KI_MAX")
print(KI_MAX if Ki > KI_MAX else Ki)
EOF
)
        Kd_final=$(python3 - <<EOF
Kp = float("$Kp_final")
Kd = Kp / 50.0
KD_MAX = float("$KD_MAX")
print(KD_MAX if Kd > KD_MAX else Kd)
EOF
)
    fi

    ###############################################
    # 3) RISULTATI
    ###############################################
    echo -e "\n=== RISULTATI PER VELOCITÀ ${speed} rad/s ==="
    echo "Metodo: $mode"
    echo
    echo "Parametri PID suggeriti:"
    echo "  Kp = $Kp_final"
    echo "  Ki = $Ki_final"
    echo "  Kd = $Kd_final"

    # Log finale
    echo "FINAL MODE=${mode} SPEED=${speed} Kp=${Kp_final} Ki=${Ki_final} Kd=${Kd_final}" >> "$LOG_FILE"

    send_cmd 1 0 0 0 0

    echo -e "\n>>> Imposta questi valori sul microcontrollore."
}

###############################################
# 🔧 TEST PID AUTOMATICO SU VELOCITÀ -13 → +13 rad/s
###############################################
test_pid_sweep_single_motor() {

    echo -e "\n=== Test PID automatico su un motore ==="
    echo "  1) Front Left"
    echo "  2) Front Right"
    echo "  3) Rear Left"
    echo "  4) Rear Right"
    read -rp "Motore (1-4): " motor_sel

    case "$motor_sel" in
        1) M="FL" ;;
        2) M="FR" ;;
        3) M="RL" ;;
        4) M="RR" ;;
        *) echo "Motore non valido"; return ;;
    esac

    # Sweep: -13..-1, 1..13
    SPEEDS=$(seq -13 -1 && seq 1 13)
    TEST_DURATION=2

    declare -A RESULT
    declare -A ERROR

    for S in $SPEEDS; do
        # Target per i 4 motori
        FL=0; FR=0; RL=0; RR=0
        case "$M" in
            FL) FL=$S ;;
            FR) FR=$S ;;
            RL) RL=$S ;;
            RR) RR=$S ;;
        esac

        # 1) Comando
        send_cmd "$TEST_DURATION" "$FL" "$FR" "$RL" "$RR"

        # 2) Misura (aggiorna CSV)
        python3 wheel_monitor.py \
            --duration "$TEST_DURATION" \
            --target $FL $FR $RL $RR \
            --motor "$M" >/dev/null 2>&1

        # 3) Leggi ultima riga CSV
        LAST_LINE=$(tail -n 1 mecanum_test_log.csv)
        LAST_ERR=$(echo "$LAST_LINE" | awk -F',' '
            {
                if ("'"$M"'"=="FL") print $(11);
                if ("'"$M"'"=="FR") print $(12);
                if ("'"$M"'"=="RL") print $(13);
                if ("'"$M"'"=="RR") print $(14);
            }')

        # Se non numerico → NOK e errore vuoto
        if ! python3 - <<EOF >/dev/null 2>&1
float("$LAST_ERR")
EOF
        then
            RESULT["$M,$S"]="NOK"
            ERROR["$M,$S"]=""
            send_cmd 1 0 0 0 0
            continue
        fi

        ERROR["$M,$S"]=$LAST_ERR

        # 4) Classificazione (soglie per odometria)
        CLASS=$(python3 - <<EOF
e = float("$LAST_ERR")
ae = abs(e)
if ae < 8:
    print("OK")
elif ae < 15:
    print("ACC")
else:
    print("NOK")
EOF
)
        RESULT["$M,$S"]=$CLASS

        # 5) Stop motore
        send_cmd 1 0 0 0 0
    done

    ###############################################
    # REPORT FINALE (classi + errori %)
    ###############################################
    echo -e "\n==================== REPORT FINALE ====================\n"

    # Header velocità
    printf -- "Motor |"
    for S in $SPEEDS; do
        printf -- " %4d" "$S"
    done
    printf -- "\n"

    # Separatore allineato alle colonne
    printf -- "------+"
    for S in $SPEEDS; do
        printf -- "-----"
    done
    printf -- "\n"

    # "Colorize" senza colori, ma larghezza fissa
    colorize() {
        local text="$1"
        case "$text" in
            OK)  echo "OK"  ;;
            ACC) echo "ACC" ;;
            NOK) echo "NOK" ;;
            *)   echo "??"  ;;
        esac
    }

    # Riga classi
    printf -- "%4s  |" "$M"
    for S in $SPEEDS; do
        RAW="${RESULT["$M,$S"]}"
        CELL=$(colorize "$RAW")
        printf -- " %4s" "$CELL"
    done
    printf -- "\n"

    # Riga errori percentuali (valori assoluti, 1 decimale)
    printf -- "err%% |"
    for S in $SPEEDS; do
        E="${ERROR["$M,$S"]}"
        if [ -z "$E" ]; then
            CELL="-"
        else
            CELL=$(python3 - <<EOF
e = float("$E")
print(f"{abs(e):.1f}")
EOF
)
        fi
        printf -- " %4s" "$CELL"
    done
    printf -- "\n"
}




echo -e "=== Avvio test Mecanum Velocity Controller ==="

while true; do
    echo -e "\nScegli un test:"
    echo "  1) Test completo (solo comandi)"
    echo "  2) Test motore singolo (con misura)"
    echo "  3) Tuning PID MANUALE (Ziegler–Nichols)"
    echo "  4) Test PID automatico (1→13 rad/s)"
    echo "  0) Esci"
    read -rp "> " choice

    case "$choice" in
        1) send_cmd 3 1 1 1 1
           send_cmd 3 -1 -1 -1 -1
           send_cmd 3 1 -1 1 -1
           send_cmd 3 -1 1 -1 1
           send_cmd 3 1 -1 -1 1
           send_cmd 3 -1 1 1 -1
           send_cmd 3 0 0 0 0 ;;
        2) test_single_motor ;;
        3) test_pid_manual ;;
        4) test_pid_sweep_single_motor ;;
        0) echo "Uscita."; exit 0 ;;
        *) echo "Scelta non valida." ;;
    esac
done
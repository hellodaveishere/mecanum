#!/bin/bash

# ============================
# 🧪 Test robot Mecanum ROS 2 — selezione da riga di comando
# ============================
# Uso: ./test_mecanum.sh <numero>
# 1 = motors (velocità in m/s e rad/s)
# 2 = servos (posizione in radianti)
# 3 = sonar
# 4 = imu
# 5 = all

if [ -z "$1" ]; then
  echo "⚠️ Devi specificare un numero:"
  echo "1 = motors, 2 = servos, 3 = sonar, 4 = imu, 5 = all"
  exit 1
fi

case $1 in
  1) TEST_MODE="motors" ;;
  2) TEST_MODE="servos" ;;
  3) TEST_MODE="sonar" ;;
  4) TEST_MODE="imu" ;;
  5) TEST_MODE="all" ;;
  *) echo "⚠️ Numero non valido. Usa 1-5."; exit 1 ;;
esac

echo "🤖 Avvio test Mecanum con modalità: $TEST_MODE"
echo ""

if [ "$TEST_MODE" = "motors" ]; then
  echo "🚗 Test ruote — diverse velocità e direzioni"
  echo "Unità: linear.x in m/s, angular.z in rad/s"

  echo "➡️ Avanti lento (0.2 m/s)"
  ros2 topic pub /mecanum_velocity_controller/commands geometry_msgs/msg/Twist "linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" --once

  echo "➡️ Avanti veloce (0.8 m/s)"
  ros2 topic pub /mecanum_velocity_controller/commands geometry_msgs/msg/Twist "linear:
    x: 0.8
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" --once

  echo "➡️ Rotazione a sinistra (0.5 rad/s)"
  ros2 topic pub /mecanum_velocity_controller/commands geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5" --once

  echo "➡️ Diagonale avanti-destra (0.3 m/s, -0.3 m/s)"
  ros2 topic pub /mecanum_velocity_controller/commands geometry_msgs/msg/Twist "linear:
    x: 0.3
    y: -0.3
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" --once

  echo "✅ Test ruote completato."

elif [ "$TEST_MODE" = "servos" ]; then
  echo "🎯 Test servo — diverse posizioni pan/tilt"
  echo "Unità: posizione in radianti (rad)"

  echo "➡️ Centro (0 rad, 0 rad)"
  ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]" --once

  echo "➡️ Estremo positivo (+1 rad, +1 rad ≈ +57°)"
  ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0]" --once

  echo "➡️ Estremo negativo (-1 rad, -1 rad ≈ -57°)"
  ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [-1.0, -1.0]" --once

  echo "➡️ Posizione intermedia (0.5 rad ≈ 29°, -0.5 rad ≈ -29°)"
  ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]" --once

  echo "✅ Test servo completato."

elif [ "$TEST_MODE" = "sonar" ]; then
  echo "📡 Test sonar — richiede invio UART dal microcontroller"
  echo "✅ Test sonar pronto per verifica via RViz o topic echo."

elif [ "$TEST_MODE" = "imu" ]; then
  echo "🧭 Test IMU — richiede invio UART dal microcontroller"
  echo "✅ Test IMU pronto per verifica via RViz o topic echo."

elif [ "$TEST_MODE" = "all" ]; then
  echo "🧪 Test completo — ruote + servo"
  echo "Unità: ruote in m/s e rad/s, servo in rad"

  echo "➡️ Ruote avanti medio (0.4 m/s, 0.2 rad/s)"
  ros2 topic pub /mecanum_velocity_controller/commands geometry_msgs/msg/Twist "linear:
    x: 0.4
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2" --once

  echo "➡️ Servo pan/tilt intermedio (0.5 rad ≈ 29°, -0.3 rad ≈ -17°)"
  ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3]" --once

  echo "✅ Test completo eseguito."
fi

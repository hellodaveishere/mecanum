#!/bin/bash

# ============================
# 🧪 Test robot Mecanum ROS 2 — selezione interna
# ============================

# 🔧 Seleziona cosa testare modificando questa variabile:
# Opzioni valide: motors, servos, imu, sonar, all
TEST_MODE="motors"

echo "🤖 Avvio test Mecanum con modalità: $TEST_MODE"
echo ""

if [ "$TEST_MODE" = "motors" ]; then
  echo "🚗 Test ruote — diverse velocità e direzioni"

  echo "➡️ Avanti lento"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" --once

  echo "➡️ Avanti veloce"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.8
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" --once

  echo "➡️ Rotazione a sinistra"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5" --once

  echo "➡️ Diagonale avanti-destra"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
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

  echo "➡️ Centro"
  ros2 topic pub /pan_controller/commands std_msgs/msg/Float64 "data: 0.0" --once
  ros2 topic pub /tilt_controller/commands std_msgs/msg/Float64 "data: 0.0" --once

  echo "➡️ Estremo positivo"
  ros2 topic pub /pan_controller/commands std_msgs/msg/Float64 "data: 1.0" --once
  ros2 topic pub /tilt_controller/commands std_msgs/msg/Float64 "data: 1.0" --once

  echo "➡️ Estremo negativo"
  ros2 topic pub /pan_controller/commands std_msgs/msg/Float64 "data: -1.0" --once
  ros2 topic pub /tilt_controller/commands std_msgs/msg/Float64 "data: -1.0" --once

  echo "➡️ Posizione intermedia"
  ros2 topic pub /pan_controller/commands std_msgs/msg/Float64 "data: 0.5" --once
  ros2 topic pub /tilt_controller/commands std_msgs/msg/Float64 "data: -0.5" --once

  echo "✅ Test servo completato."

elif [ "$TEST_MODE" = "sonar" ]; then
  echo "📡 Test sonar — richiede invio UART dal microcontroller"
  echo "➡️ Verifica che il microcontroller invii pacchetti tipo: SON,1.23,0.98,1.10"
  echo "➡️ I dati saranno pubblicati automaticamente dai range_sensor_broadcaster"
  echo "✅ Test sonar pronto per verifica via RViz o topic echo."

elif [ "$TEST_MODE" = "imu" ]; then
  echo "🧭 Test IMU — richiede invio UART dal microcontroller"
  echo "➡️ Verifica che il microcontroller invii pacchetti tipo: IMU,qx,qy,qz,qw,wx,wy,wz,ax,ay,az"
  echo "➡️ I dati saranno pubblicati automaticamente da imu_sensor_broadcaster"
  echo "✅ Test IMU pronto per verifica via RViz o topic echo."

elif [ "$TEST_MODE" = "all" ]; then
  echo "🧪 Test completo — ruote + servo"

  echo "➡️ Ruote avanti medio"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
    x: 0.4
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2" --once

  echo "➡️ Servo pan/tilt intermedio"
  ros2 topic pub /pan_controller/commands std_msgs/msg/Float64 "data: 0.5" --once
  ros2 topic pub /tilt_controller/commands std_msgs/msg/Float64 "data: -0.3" --once

  echo "➡️ Verifica sonar e IMU via UART"
  echo "✅ Test completo eseguito."

else
  echo "⚠️ Modalità TEST_MODE non valida: '$TEST_MODE'"
  echo "Modifica la variabile TEST_MODE in alto nel file."
fi
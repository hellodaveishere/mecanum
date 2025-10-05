cmake_minimum_required(VERSION 3.8)  # Versione minima di CMake richiesta
project(mecanum_base)                # Nome del pacchetto ROS 2

# =========================
# 1. TROVA I PACCHETTI NECESSARI
# =========================
# Questi comandi cercano i pacchetti ROS 2 richiesti per compilare e linkare il progetto

find_package(ament_cmake REQUIRED)           # Sistema di build ROS 2
find_package(rclcpp REQUIRED)                # API C++ per ROS 2
find_package(rclcpp_lifecycle REQUIRED)      # Supporto per nodi lifecycle (hardware interface)
find_package(pluginlib REQUIRED)             # Per caricare plugin dinamici
find_package(hardware_interface REQUIRED)    # Interfaccia base per ros2_control
find_package(controller_manager REQUIRED)    # Per gestire i controller
find_package(controller_interface REQUIRED)  # Base per broadcaster e controller ROS 2
find_package(sensor_msgs REQUIRED)           # Messaggi per IMU, sonar, ecc.
find_package(geometry_msgs REQUIRED)         # Twist, Pose, Vector3, ecc.
find_package(nav_msgs REQUIRED)              # Messaggi di odometria
find_package(tf2_ros REQUIRED)               # Trasformazioni tra frame
find_package(std_msgs REQUIRED)              # Messaggi semplici (es. Float64 per servo)

# =========================
# 2. LIBRERIA PLUGIN HARDWARE
# =========================
# Compila la classe MecanumSystem come libreria condivisa da caricare via pluginlib

add_library(mecanum_system SHARED
  src/mecanum_system.cpp
)

target_compile_features(mecanum_system PUBLIC cxx_std_17)  # Usa C++17

ament_target_dependencies(mecanum_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
  std_msgs
)

# =========================
# 3. NODI DI UTILITÀ
# =========================
# Compila i nodi custom per comandi e odometria

add_executable(mecanum_cmd_node src/mecanum_cmd_node.cpp)
target_compile_features(mecanum_cmd_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_cmd_node
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(mecanum_odom_node src/mecanum_odom_node.cpp)
target_compile_features(mecanum_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_odom_node
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
)

# =========================
# 4. INCLUDE COMUNI
# =========================
# Specifica dove trovare gli header durante la compilazione e installazione

set(ALL_TARGETS
  mecanum_system
  mecanum_cmd_node
  mecanum_odom_node
)

foreach(tgt IN LISTS ALL_TARGETS)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>     # Durante build
    $<INSTALL_INTERFACE:include>                               # Dopo installazione
  )
endforeach()

# =========================
# 5. PLUGIN ROS2_CONTROL
# =========================
# Esporta il file XML che descrive il plugin hardware per ros2_control

pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugin.xml)

# =========================
# 6. INSTALLAZIONE
# =========================
# Specifica cosa installare e dove

install(TARGETS
  mecanum_system
  mecanum_cmd_node
  mecanum_odom_node
  ARCHIVE DESTINATION lib                      # Librerie statiche (se presenti)
  LIBRARY DESTINATION lib                      # Librerie condivise
  RUNTIME DESTINATION lib/${PROJECT_NAME}      # Eseguibili
)

install(DIRECTORY include/ DESTINATION include)                         # Header
install(FILES mecanum_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})  # Plugin XML
install(DIRECTORY urdf config launch rviz DESTINATION share/${PROJECT_NAME})  # Risorse

# =========================
# 7. MACRO FINALE
# =========================
# Registra il pacchetto con ament per ROS 2

ament_package()
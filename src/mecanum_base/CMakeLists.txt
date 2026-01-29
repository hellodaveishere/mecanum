cmake_minimum_required(VERSION 3.8)
project(mecanum_base)

# ============================================================
# 1. TROVA I PACCHETTI NECESSARI
# ============================================================
# Questi pacchetti forniscono le funzionalità ROS 2 e le dipendenze per nodi, messaggi, plugin e interfacce
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Necessario per generare messaggi/servizi custom
find_package(std_srvs REQUIRED)
find_package(controller_interface REQUIRED)


# ============================================================
# 2. GENERAZIONE INTERFACCE ROSIDL (.srv)
# ============================================================
# Genera il servizio CalibrationInput.srv e le relative interfacce
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrationInput.srv"
  DEPENDENCIES std_msgs
)

# ============================================================
# 3. LIBRERIA HARDWARE INTERFACE (plugin ros2_control)
# ============================================================
# Crea una libreria condivisa per l'interfaccia hardware della base Mecanum
add_library(mecanum_system SHARED
  src/mecanum_system.cpp
)
target_compile_features(mecanum_system PUBLIC cxx_std_17)

# Collega le dipendenze necessarie alla libreria
ament_target_dependencies(mecanum_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

# ============================================================
# LIBRERIA BROADCASTER ESTOP
# ============================================================
add_library(mecanum_base_estop_state_broadcaster SHARED
  src/estop_state_broadcaster.cpp
)
target_compile_features(mecanum_base_estop_state_broadcaster PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_base_estop_state_broadcaster
  rclcpp
  controller_interface
  std_msgs
)


# ============================================================
# 4. NODI DI UTILITÀ (eseguibili ROS 2)
# ============================================================

# Nodo per inviare comandi manuali alla base
add_executable(mecanum_cmd_node src/mecanum_cmd_node.cpp)
target_compile_features(mecanum_cmd_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_cmd_node
  rclcpp
  geometry_msgs
)

# Nodo per calcolare e pubblicare odometria
add_executable(mecanum_odom_node src/mecanum_odom_node.cpp)
target_compile_features(mecanum_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_odom_node
  rclcpp
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
)

# Nodo per convertire messaggi rosout in stringhe leggibili
add_executable(rosout_relay_node src/rosout_relay_node.cpp)
ament_target_dependencies(rosout_relay_node
  rclcpp
  std_msgs
  rcl_interfaces
)

# Nodo per calibrazione delle ruote
add_executable(calibration_node src/calibration_node.cpp)
target_compile_features(calibration_node PUBLIC cxx_std_17)
ament_target_dependencies(calibration_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Nodo per gestire lo stop di emergenza da parte di rpi pico
add_executable(estop_manager_node
  src/estop_manager_node.cpp
)
target_compile_features(estop_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(estop_manager_node
  rclcpp std_srvs controller_manager_msgs
)

# ============================================================
# 5. COLLEGAMENTO ALLE INTERFACCE ROSIDL
# ============================================================
# Collega il nodo calibration_node alle interfacce generate dal servizio
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(calibration_node "${cpp_typesupport_target}")

# ============================================================
# 6. INCLUDE COMUNI PER TUTTI I TARGET
# ============================================================
# Applica le directory di include a tutti i nodi e librerie
set(ALL_TARGETS
  mecanum_system
  mecanum_base_estop_state_broadcaster
  mecanum_cmd_node
  mecanum_odom_node
  calibration_node
  estop_manager_node
)

foreach(tgt IN LISTS ALL_TARGETS)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

# ============================================================
# 7. ESPORTAZIONE DEL PLUGIN ROS2_CONTROL
# ============================================================
# Descrive il plugin per ros2_control tramite file XML
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugin.xml)

# Esporta il plugin broadcaster
pluginlib_export_plugin_description_file(controller_interface estop_state_broadcaster_plugin.xml)


# ============================================================
# 8. INSTALLAZIONE DEI FILE
# ============================================================
# Installa librerie ed eseguibili
install(TARGETS
  mecanum_system
  mecanum_base_estop_state_broadcaster
  mecanum_cmd_node
  mecanum_odom_node
  rosout_relay_node
  calibration_node
  estop_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installa gli header
install(DIRECTORY include/ DESTINATION include)

# Installa il file XML del plugin
install(FILES
  mecanum_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Installa risorse condivise (URDF, config, launch, RViz, webserver)
install(DIRECTORY urdf config launch rviz web webserver DESTINATION share/${PROJECT_NAME})

# ============================================================
# 9. MACRO FINALE PER IL PACCHETTO
# ============================================================
# Necessario per completare la configurazione del pacchetto ROS 2
ament_package()

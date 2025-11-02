cmake_minimum_required(VERSION 3.8)
project(mecanum_base)

# =========================
# 1. TROVA I PACCHETTI NECESSARI
# =========================
find_package(ament_cmake REQUIRED)              # Sistema di build ROS 2
find_package(rclcpp REQUIRED)                   # Nodi ROS 2 e logging
find_package(rclcpp_lifecycle REQUIRED)         # Lifecycle node per hardware interface
find_package(pluginlib REQUIRED)                # Per registrare e caricare plugin
find_package(hardware_interface REQUIRED)       # API ros2_control per SystemInterface
find_package(controller_manager REQUIRED)       # Per gestire i controller
find_package(sensor_msgs REQUIRED)              # Per usare sensor_msgs/Range

# Dipendenze aggiuntive per i nodi di utilità
find_package(tf2_ros REQUIRED)                  # Per trasformazioni TF
find_package(geometry_msgs REQUIRED)            # Per Twist e Pose
find_package(nav_msgs REQUIRED)                 # Per Odometry
find_package(std_msgs REQUIRED)                 # Per messaggi generici
find_package(rcl_interfaces REQUIRED)           # Per introspezione parametri

# =========================
# 2. INTERFACCE PERSONALIZZATE
# =========================

# Trova il generatore di interfacce ROS 2
find_package(rosidl_default_generators REQUIRED)

# Genera interfacce dal file srv/CalibrationInput.srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrationInput.srv"
)

# =========================
# 3. LIBRERIE PLUGIN
# =========================

# Libreria hardware interface per ros2_control
add_library(mecanum_system SHARED
  src/mecanum_system.cpp
)

# Imposta lo standard C++ per la libreria
target_compile_features(mecanum_system PUBLIC cxx_std_17)

# Dipendenze per la libreria hardware
ament_target_dependencies(mecanum_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

# =========================
# 4. NODI DI UTILITÀ
# =========================

# Nodo per inviare comandi manuali e gestire calibrazione
add_executable(mecanum_cmd_node src/mecanum_cmd_node.cpp)
target_compile_features(mecanum_cmd_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_cmd_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Collegamento alle interfacce generate (servizi)
rosidl_target_interfaces(mecanum_cmd_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

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

# Nodo per convertire rosout message in stringa
add_executable(rosout_relay_node src/rosout_relay_node.cpp)
ament_target_dependencies(rosout_relay_node
  rclcpp
  std_msgs
  rcl_interfaces
)

# =========================
# 5. INCLUDE COMUNI
# =========================

# Applica le directory di include a tutti i target
set(ALL_TARGETS
  mecanum_system
  mecanum_cmd_node
  mecanum_odom_node
)

foreach(tgt IN LISTS ALL_TARGETS)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

# =========================
# 6. PLUGIN ROS2_CONTROL
# =========================

# Esporta il file XML del plugin hardware
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugin.xml)

# =========================
# 7. INSTALLAZIONE
# =========================

# Installa le librerie e gli eseguibili
install(TARGETS
  mecanum_system
  mecanum_cmd_node
  mecanum_odom_node
  rosout_relay_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installa gli header
install(DIRECTORY include/ DESTINATION include)

# Installa i file plugin XML
install(FILES
  mecanum_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Installa risorse condivise (URDF, config, launch, RViz, webserver)
install(DIRECTORY urdf config launch rviz webserver DESTINATION share/${PROJECT_NAME})

# =========================
# 8. MACRO FINALE
# =========================

ament_package()

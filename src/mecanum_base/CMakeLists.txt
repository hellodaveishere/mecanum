cmake_minimum_required(VERSION 3.8)
project(mecanum_base)

# =========================
# 1. TROVA I PACCHETTI NECESSARI
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                # logging e utilità base
find_package(rclcpp_lifecycle REQUIRED)      # supporto lifecycle per SystemInterface
find_package(pluginlib REQUIRED)             # caricamento dinamico plugin
find_package(hardware_interface REQUIRED)    # API ros2_control
find_package(controller_manager REQUIRED)    # per caricare/attivare plugin
find_package(sensor_msgs REQUIRED)           # per coerenza con imu_broadcaster

# Dipendenze aggiuntive solo se servono nodi extra
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# =========================
# 2. LIBRERIA PLUGIN HARDWARE
# =========================
# Libreria condivisa che implementa l'interfaccia hardware
add_library(mecanum_system SHARED
  src/mecanum_system.cpp
)

target_compile_features(mecanum_system PUBLIC cxx_std_17)

ament_target_dependencies(mecanum_system
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

# =========================
# 3. NODI DI UTILITÀ (opzionali)
# =========================
# Nodo per inviare comandi manuali
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

# =========================
# 4. INCLUDE COMUNI
# =========================
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
# 5. PLUGIN ROS2_CONTROL
# =========================
# Esporta il file XML che descrive il plugin hardware
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugin.xml)

# =========================
# 6. INSTALLAZIONE
# =========================
install(TARGETS
  mecanum_system
  mecanum_cmd_node
  mecanum_odom_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Installa header, file plugin e risorse (urdf, config, launch, rviz)
install(DIRECTORY include/ DESTINATION include)
install(FILES mecanum_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf config launch rviz DESTINATION share/${PROJECT_NAME})

# =========================
# 7. MACRO FINALE
# =========================
ament_package()

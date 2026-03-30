cmake_minimum_required(VERSION 3.8)
project(mecanum_base)

# ============================================================
# 1. TROVA I PACCHETTI NECESSARI
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Solo se usi messaggi Nav2 nel client
find_package(nav2_msgs REQUIRED)


# ============================================================
# 2. GENERAZIONE INTERFACCE ROSIDL (.srv)
# ============================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrationInput.srv"
  DEPENDENCIES std_msgs
)

# ============================================================
# 3. LIBRERIA HARDWARE INTERFACE (plugin ros2_control)
# ============================================================
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
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# 4. LIBRERIA BROADCASTER ESTOP
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
# 5. NODI DI UTILITÀ
# ============================================================

add_executable(mecanum_cmd_node src/mecanum_cmd_node.cpp)
target_compile_features(mecanum_cmd_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_cmd_node
  rclcpp
  geometry_msgs
)

add_executable(mecanum_odom_node src/mecanum_odom_node.cpp)
target_compile_features(mecanum_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(mecanum_odom_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(rosout_relay_node src/rosout_relay_node.cpp)
ament_target_dependencies(rosout_relay_node
  rclcpp
  std_msgs
  rcl_interfaces
)

add_executable(calibration_node src/calibration_node.cpp)
target_compile_features(calibration_node PUBLIC cxx_std_17)
ament_target_dependencies(calibration_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(estop_manager_node src/estop_manager_node.cpp)
target_compile_features(estop_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(estop_manager_node
  rclcpp
  std_srvs
  controller_manager_msgs
)

add_executable(my_nav_client src/my_nav_client.cpp)
target_compile_features(my_nav_client PUBLIC cxx_std_17)
ament_target_dependencies(my_nav_client
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# 6. COLLEGAMENTO ALLE INTERFACCE ROSIDL
# ============================================================
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(calibration_node "${cpp_typesupport_target}")

# ============================================================
# 7. INCLUDE COMUNI
# ============================================================
set(ALL_TARGETS
  mecanum_system
  mecanum_base_estop_state_broadcaster
  mecanum_cmd_node
  mecanum_odom_node
  calibration_node
  estop_manager_node
  my_nav_client
)

foreach(tgt IN LISTS ALL_TARGETS)
  target_include_directories(${tgt} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

# ============================================================
# 8. ESPORTAZIONE PLUGIN ROS2_CONTROL
# ============================================================
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface estop_state_broadcaster_plugin.xml)

# ============================================================
# 9. INSTALLAZIONE
# ============================================================
install(TARGETS
  mecanum_system
  mecanum_base_estop_state_broadcaster
  mecanum_cmd_node
  mecanum_odom_node
  rosout_relay_node
  calibration_node
  estop_manager_node
  my_nav_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(FILES
  mecanum_hardware_plugin.xml
  estop_state_broadcaster_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ============================================================
# 10. MACRO FINALE
# ============================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()

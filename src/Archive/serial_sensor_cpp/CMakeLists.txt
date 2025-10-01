cmake_minimum_required(VERSION 3.5)
project(serial_sensor_cpp)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(serial             REQUIRED)
find_package(nlohmann_json      REQUIRED)

add_executable(serial_sensor_node src/serial_sensor_node.cpp)

ament_target_dependencies(
  serial_sensor_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_updater
  serial
  nlohmann_json
)

install(
  TARGETS serial_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

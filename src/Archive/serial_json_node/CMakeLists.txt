cmake_minimum_required(VERSION 3.8)
project(serial_json_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

# Optional: local serial lib
set(SERIAL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../serial-ros2")


add_executable(serial_json_node src/serial_json_node.cpp)

#include_directories(${SERIAL_LIB_DIR}/include include)
include_directories(
  include
)

target_include_directories(serial_json_node
  PUBLIC
    ${SERIAL_LIB_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)


target_link_libraries(serial_json_node
  ${SERIAL_LIB_DIR}/build/libserial.a
  Threads::Threads
)

ament_target_dependencies(serial_json_node
  rclcpp
  std_msgs
  sensor_msgs
  nlohmann_json
)

target_compile_features(serial_json_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS serial_json_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/ 
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

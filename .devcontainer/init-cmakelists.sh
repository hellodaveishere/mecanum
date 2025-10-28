#!/usr/bin/env bash

cat > /home/ws/src/serial/CMakeLists.txt/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(serial VERSION 1.2.1 LANGUAGES CXX)

find_package(Threads REQUIRED)

add_library(serial
  src/serial.cpp
)
target_include_directories(serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial
  PRIVATE Threads::Threads
)

# --- Generazione pacchetti CMake per find_package() ---
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/serialConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(TARGETS serial
  EXPORT serialTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/serial DESTINATION include)

install(EXPORT serialTargets
  FILE serialTargets.cmake
  NAMESPACE serial::
  DESTINATION lib/cmake/serial
)
install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/serialConfigVersion.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/serialConfig.cmake.in"
  DESTINATION lib/cmake/serial
)
EOF

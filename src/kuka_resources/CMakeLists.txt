cmake_minimum_required(VERSION 3.8)
project(kuka_resources)

# Check for compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament package system
find_package(ament_cmake REQUIRED)

# Install urdf directory
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Ament package finalization
ament_package()
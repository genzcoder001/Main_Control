cmake_minimum_required(VERSION 3.8)
project(rado_control_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find only the ament_cmake dependency, which is required for the build type.
find_package(ament_cmake REQUIRED)

# Install the Python executable scripts. This is the correct way to handle them.
install(
  PROGRAMS
  scripts/state_manager_node.py
  scripts/coordinate_follower_node.py

  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch and config directories. This remains the same.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(safety_supervisor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Add executable
add_executable(safety_supervisor_node
  src/safety_supervisor.cpp
)

# Enable C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Link and include deps
ament_target_dependencies(safety_supervisor_node
  rclcpp
  std_msgs
  std_srvs
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  diagnostic_msgs
)

# Install targets
install(TARGETS
  safety_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch/")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# Install config files if any
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  diagnostic_msgs
)

ament_package()

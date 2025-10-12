cmake_minimum_required(VERSION 3.8)
project(state_machine)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}_node
  src/state_machine_node.cpp
)

# Link and dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME})
endif()

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

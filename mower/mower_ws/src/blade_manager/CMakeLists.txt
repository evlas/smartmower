cmake_minimum_required(VERSION 3.8)
project(blade_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(blade_manager_node src/blade_manager_node.cpp)
ament_target_dependencies(blade_manager_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS blade_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

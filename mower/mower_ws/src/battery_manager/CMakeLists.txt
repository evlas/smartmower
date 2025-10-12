cmake_minimum_required(VERSION 3.8)
project(battery_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(battery_manager_node src/battery_manager_node.cpp)
ament_target_dependencies(battery_manager_node rclcpp std_msgs sensor_msgs diagnostic_msgs)

install(TARGETS
  battery_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

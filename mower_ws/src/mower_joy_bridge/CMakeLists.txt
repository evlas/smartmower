cmake_minimum_required(VERSION 3.8)
project(mower_joy_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joy_bridge_node src/joy_bridge_node.cpp)
ament_target_dependencies(joy_bridge_node rclcpp sensor_msgs std_msgs)

install(TARGETS joy_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()

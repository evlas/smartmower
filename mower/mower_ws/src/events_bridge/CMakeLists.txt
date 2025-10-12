cmake_minimum_required(VERSION 3.8)
project(events_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(events_bridge_node src/events_bridge_node.cpp)
ament_target_dependencies(events_bridge_node rclcpp std_msgs)

install(TARGETS events_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

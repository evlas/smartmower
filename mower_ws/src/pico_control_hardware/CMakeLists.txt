cmake_minimum_required(VERSION 3.8)
project(pico_control_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/pico_system_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
  nav_msgs
)

pluginlib_export_plugin_description_file(hardware_interface pico_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES pico_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

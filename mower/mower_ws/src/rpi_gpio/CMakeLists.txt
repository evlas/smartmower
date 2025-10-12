cmake_minimum_required(VERSION 3.8)
project(rpi_gpio)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# libgpiod (system library)
find_library(GPIOD_LIBRARY NAMES gpiod)
find_path(GPIOD_INCLUDE_DIR NAMES gpiod.h)

if(NOT GPIOD_LIBRARY OR NOT GPIOD_INCLUDE_DIR)
  message(FATAL_ERROR "libgpiod not found. Please install: sudo apt-get install -y libgpiod-dev")
endif()

include_directories(
  include
  ${GPIOD_INCLUDE_DIR}
)

add_executable(rpi_gpio_node src/rpi_gpio_node.cpp)
target_link_libraries(rpi_gpio_node ${GPIOD_LIBRARY})
ament_target_dependencies(rpi_gpio_node rclcpp std_msgs)

install(TARGETS rpi_gpio_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

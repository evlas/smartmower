cmake_minimum_required(VERSION 3.8)
project(relay_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# Aggiungi opzioni di compilazione per i warning
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Aggiungi include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Crea l'eseguibile
add_executable(${PROJECT_NAME}_node
  src/relay_manager.cpp
)

# Linka le librerie
target_link_libraries(${PROJECT_NAME}_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Installa i target
install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Installa i file di configurazione
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Aggiungi i test se necessario
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Esporta le dipendenze - deve essere l'ultima chiamata
ament_export_dependencies(rclcpp std_msgs)
ament_export_include_directories(include)

# Deve essere l'ultima istruzione
ament_package()

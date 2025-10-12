cmake_minimum_required(VERSION 3.8)
project(mower_description)

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro and launch files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Gazebo (ros_gz) models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

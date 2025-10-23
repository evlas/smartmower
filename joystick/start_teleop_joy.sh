#!/bin/bash

# Attendi che ROS2 sia pronto
sleep 2

# Source del workspace ROS2
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/mower/mower_ws/install/setup.bash

# Avvia il nodo teleop_joy
export ROS_DOMAIN_ID=0  # Se usi un domain ID specifico
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p joy_vel:=/mower/cmd_vel/manual \
  -p enable_button:=4 \
  -p axis_linear.x:=1 \
  -p scale_linear.x:=0.5 \
  -p axis_angular.yaw:=0 \
  -p scale_angular.yaw:=1.0 &

# Salva il PID
echo $! > /tmp/teleop_joy.pid

exit 0

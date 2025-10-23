#!/bin/bash

# Attendi che ROS2 sia pronto
sleep 2

# Source del workspace ROS2
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/mower/mower_ws/install/setup.bash

# Avvia il driver del joystick (joy_node)
export ROS_DOMAIN_ID=0  # Se usi un domain ID specifico
ros2 run joy joy_node --ros-args \
  -p autorepeat_rate:=20.0 \
  -p coalesce_interval:=0.02 &

# Salva il PID
echo $! > /tmp/teleop_joy.pid

exit 0

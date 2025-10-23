#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[{
                'use_stamped': True,
                'use_select': True,
                'use_sim_time': False,
                'topics.manual_cmd_vel.topic': '/mower/cmd_vel/manual',
                'topics.manual_cmd_vel.timeout': 0.5,
                'topics.manual_cmd_vel.priority': 100,
                'topics.docking_cmd_vel.topic': '/mower/cmd_vel/docking',
                'topics.docking_cmd_vel.timeout': 0.5,
                'topics.docking_cmd_vel.priority': 40,
                'topics.mowing_cmd_vel.topic': '/mower/cmd_vel/mowing',
                'topics.mowing_cmd_vel.timeout': 0.5,
                'topics.mowing_cmd_vel.priority': 30,
                'topics.undocking_cmd_vel.topic': '/mower/cmd_vel/undocking',
                'topics.undocking_cmd_vel.timeout': 0.5,
                'topics.undocking_cmd_vel.priority': 20,
                'topics.stop_cmd_vel.topic': '/mower/cmd_vel/stop',
                'topics.stop_cmd_vel.timeout': 0.5,
                'topics.stop_cmd_vel.priority': 0,
            }],
            remappings=[
                ('cmd_vel_out', 'diff_drive_controller/cmd_vel'),
            ]
        )
    ])

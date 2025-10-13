#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port = LaunchConfiguration('port', default='8765')
    address = LaunchConfiguration('address', default='0.0.0.0')
    send_buffer_limit = LaunchConfiguration('send_buffer_limit', default='500000000')  # 500MB

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': port, 'address': address, 'send_buffer_limit': send_buffer_limit}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765', description='Port for Foxglove Bridge WebSocket server'),
        DeclareLaunchArgument('address', default_value='0.0.0.0', description='Address to bind the server to (0.0.0.0 for all interfaces)'),
        DeclareLaunchArgument('send_buffer_limit', default_value='500000000', description='Send buffer limit in bytes (increased to 500MB)'),
        foxglove_bridge_node,
    ])

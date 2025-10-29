#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ekf_local_config = PathJoinSubstitution([
        FindPackageShare('mower_bringup'),
        'config',
        'ekf_local.yaml'
    ])

    ekf_global_config = PathJoinSubstitution([
        FindPackageShare('mower_bringup'),
        'config',
        'ekf_global.yaml'
    ])

    # EKF locale: odom + IMU -> /odometry/local, TF odom->base_link
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ],
    )

    # NavSat transform: IMU + GPS + odom local -> /odometry/gps
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_global_config],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
            ('/imu', '/imu/data')
        ],
    )

    # EKF globale: /odometry/local + /odometry/gps -> /odometry/global, TF map->odom
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config],
        remappings=[
            ('/odometry/filtered', '/odometry/global')
        ],
    )

    return LaunchDescription([
        ekf_local,
        navsat,
        ekf_global,
    ])

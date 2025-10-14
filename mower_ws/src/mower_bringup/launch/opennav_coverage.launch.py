#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        get_package_share_directory('mower_bringup'), 'config', 'coverage_params.yaml'))

    bringup_dir = get_package_share_directory('mower_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'nav2.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'namespace': '',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
            'slam': 'False',              # usiamo copertura, non SLAM
            'use_localization': 'True',   # localizzazione attiva
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('mower_bringup'), 'config', 'coverage_params.yaml')),
        nav2_launch,
    ])

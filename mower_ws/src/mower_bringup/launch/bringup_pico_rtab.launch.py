#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_control = LaunchConfiguration('enable_control', default='true')
    device = LaunchConfiguration('device', default='/dev/ttyAMA0')
    baud = LaunchConfiguration('baud', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='camera')
    map_file = LaunchConfiguration('map', default=os.path.join(os.path.expanduser('~'), 'mower', 'mower_ws', 'maps', 'map.yaml'))
    localization = LaunchConfiguration('localization', default='true')

    # Path diretto al launch file di pico_control_hardware (COMPLETO)
    pico_control_path = os.path.join(
        get_package_share_directory('pico_control_hardware'),
        'launch',
        'pico_control_hardware.launch.py'
    )

    # Includi il launch file di pico_control_hardware (con hardware completo)
    pico_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pico_control_path]),
        launch_arguments=[
            ("port", device),
            ("baud", baud),
        ]
    )

    # Altri nodi del mower (oltre al controllo)
    bringup_dir = get_package_share_directory('mower_bringup')
    ublox_yaml = os.path.join(bringup_dir, 'config', 'ublox_params.yaml')
    static_transform_publisher_yaml = os.path.join(bringup_dir, 'config', 'static_transform_publisher.yaml')
    battery_manager_yaml = os.path.join(bringup_dir, 'config', 'battery_manager.yaml')
    blade_manager_yaml = os.path.join(bringup_dir, 'config', 'blade_manager.yaml')
    relay_manager_yaml = os.path.join(bringup_dir, 'config', 'relay_manager.yaml')
    rpi_gpio_yaml = os.path.join(bringup_dir, 'config', 'rpi_gpio.yaml')
    safety_supervisor_yaml = os.path.join(bringup_dir, 'config', 'safety_supervisor.yaml')
    camera_node_yaml = os.path.join(bringup_dir, 'config', 'camera_node.yaml')
    state_machine_yaml = os.path.join(bringup_dir, 'config', 'state_machine.yaml')
    events_bridge_yaml = os.path.join(bringup_dir, 'config', 'events_bridge.yaml')

    # RTAB-Map and Nav2 paths
    rtabmap_config = os.path.join(bringup_dir, 'config', 'rtabmap.yaml')
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    nav2_bt = os.path.join(get_package_share_directory('nav2_bringup'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    nav2_config = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    # Camera static TF and node
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        parameters=[static_transform_publisher_yaml],
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', frame_id],
        output='screen'
    )

    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[camera_node_yaml],
        remappings=[
            ('/image', '/camera/image_raw'),
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )

    # ublox GPS node (serial on /dev/ttyAMA1 @115200 set via YAML)
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[ublox_yaml],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/fix_velocity', '/gps/fix_velocity'),
            ('/navstatus', '/gps/navstatus'),
        ]
    )

    # Battery Manager node
    battery_manager_node = Node(
        package='battery_manager',
        executable='battery_manager_node',
        name='battery_manager',
        output='screen',
        parameters=[battery_manager_yaml]
    )

    # Blade Manager node
    blade_manager_node = Node(
        package='blade_manager',
        executable='blade_manager_node',
        name='blade_manager',
        output='screen',
        parameters=[blade_manager_yaml]
    )

    # Relay Manager node
    relay_manager_node = Node(
        package='relay_manager',
        executable='relay_manager_node',
        name='relay_manager',
        output='screen',
        parameters=[relay_manager_yaml]
    )

    # RPI GPIO node
    rpi_gpio_node = Node(
        package='rpi_gpio',
        executable='rpi_gpio_node',
        name='rpi_gpio',
        output='screen',
        parameters=[rpi_gpio_yaml]
    )

    # Safety Supervisor node
    safety_supervisor_node = Node(
        package='safety_supervisor',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        output='screen',
        parameters=[safety_supervisor_yaml]
    )

    # State Machine node
    sm_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='mower_state_machine',
        output='screen',
        parameters=[state_machine_yaml]
    )

    # Events Bridge node
    events_bridge_node = Node(
        package='events_bridge',
        executable='events_bridge_node',
        name='events_bridge',
        output='screen',
        parameters=[events_bridge_yaml]
    )

    # Nav2 Navigation for navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
            'namespace': '',
            'use_composition': 'False',
            'container_name': 'nav2_container',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items()
    )

    # Map server for loading maps (required for localization)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': ''}]
    )

    # Collision Monitor (temporarily disabled for stability)
    # collision_monitor_node = Node(
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     output='screen',
    #     parameters=[nav2_params],
    #     remappings=[
    #         ('cmd_vel_in', 'cmd_vel_nav'),
    #         ('cmd_vel_out', 'cmd_vel'),
    #     ]
    # )

    # Velocity Smoother (temporarily disabled for stability)
    # velocity_smoother_node = Node(
    #     package='nav2_velocity_smoother',
    #     executable='velocity_smoother',
    #     name='velocity_smoother',
    #     output='screen',
    #     parameters=[nav2_params],
    #     remappings=[
    #         ('cmd_vel', 'cmd_vel_nav'),
    #         ('cmd_vel_smoothed', 'cmd_vel'),
    #     ]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('device', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='camera', description='TF frame id for the images'),
        DeclareLaunchArgument('map', default_value=os.path.join(os.path.expanduser('~'), 'mower', 'mower_ws', 'maps', 'map.yaml'), description='Path to the map file in maps directory'),
        DeclareLaunchArgument('localization', default_value='true', description='Enable localization mode'),

        # Pico control (include robot_state_publisher + controller_manager)
        pico_control_launch,

        # Altri nodi del mower
        static_tf,
        camera_node,
        ublox_node,
        battery_manager_node,
        blade_manager_node,
        relay_manager_node,
        rpi_gpio_node,
        safety_supervisor_node,
        sm_node,
        events_bridge_node,

        # RTAB-Map and Nav2
        # rtabmap_slam_node,
        nav2_launch,
        map_server_node,
        # collision_monitor_node,
        # velocity_smoother_node,
        # amcl_node,
    ])

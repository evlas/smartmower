#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_control = LaunchConfiguration('enable_control', default='true')
    port = LaunchConfiguration('port', default='/dev/ttyAMA0')
    baud = LaunchConfiguration('baud', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='camera')

    # Paths
    mower_description = get_package_share_directory('mower_description')
    xacro_file = os.path.join(mower_description, 'urdf', 'mower.urdf.xacro')

    bringup_dir = get_package_share_directory('mower_bringup')
    controllers_yaml = os.path.join(bringup_dir, 'config', 'ros2_controllers.yaml')
    ublox_yaml = os.path.join(bringup_dir, 'config', 'ublox_params.yaml')
    robot_state_publisher_yaml = os.path.join(bringup_dir, 'config', 'robot_state_publisher.yaml')
    joint_state_publisher_yaml = os.path.join(bringup_dir, 'config', 'joint_state_publisher.yaml')
    static_transform_publisher_yaml = os.path.join(bringup_dir, 'config', 'static_transform_publisher.yaml')
    battery_manager_yaml = os.path.join(bringup_dir, 'config', 'battery_manager.yaml')
    blade_manager_yaml = os.path.join(bringup_dir, 'config', 'blade_manager.yaml')
    relay_manager_yaml = os.path.join(bringup_dir, 'config', 'relay_manager.yaml')
    rpi_gpio_yaml = os.path.join(bringup_dir, 'config', 'rpi_gpio.yaml')
    safety_supervisor_yaml = os.path.join(bringup_dir, 'config', 'safety_supervisor.yaml')
    camera_node_yaml = os.path.join(bringup_dir, 'config', 'camera_node.yaml')
    state_machine_yaml = os.path.join(bringup_dir, 'config', 'state_machine.yaml')
    events_bridge_yaml = os.path.join(bringup_dir, 'config', 'events_bridge.yaml')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' port:=', port, ' baud:=', baud]),
        value_type=str
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_state_publisher_yaml, {'robot_description': robot_description}],
        output='screen'
    )

    # joint_state_publisher for non-control mode (publishes zero positions)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_publisher_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(enable_control)
    )

    # ros2_control controller_manager node
    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[controllers_yaml, {'robot_description': robot_description}],
        output='screen',
        condition=IfCondition(enable_control)
    )

    # Use spawners (Jazzy) instead of deprecated load_start_controller
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(enable_control)
    )

    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_diff',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(enable_control)
    )

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='camera', description='TF frame id for the images'),

        # robot_state_publisher PRIMA di tutto
        rsp,

        # controller_manager DOPO robot_state_publisher
        cm,
        jsp,

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

        # Delay spawners dopo controller_manager
        TimerAction(period=2.0, actions=[spawner_jsb]),
        TimerAction(period=3.0, actions=[spawner_diff]),
    ])

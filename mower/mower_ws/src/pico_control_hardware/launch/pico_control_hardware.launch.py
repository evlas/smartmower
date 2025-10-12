#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Usa l'URDF del mower con l'hardware interface di pico_control_hardware
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mower_description"), "urdf", "mower.urdf.xacro"]
            ),
            " ",
            "port:=", LaunchConfiguration("port", default="/dev/ttyAMA0"),
            " ",
            "baud:=", LaunchConfiguration("baud", default="115200"),
        ]),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}

    # Usa la configurazione completa del mower
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mower_bringup"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Controller manager con l'hardware interface completo di pico_control_hardware
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])

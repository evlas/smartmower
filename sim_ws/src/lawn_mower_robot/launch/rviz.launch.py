import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('lawn_mower_robot')

    # Process xacro to URDF string
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    urdf_xml = xacro.process_file(xacro_path).toxml()

    # Launch argument to optionally start robot_state_publisher and joint_state_publisher
    start_rsp = LaunchConfiguration('start_rsp', default='false')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_xml
        }],
        condition=IfCondition(start_rsp)
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(start_rsp)
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'lawn_mower.rviz')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rsp',
            default_value='false',
            description='If true, also start robot_state_publisher and joint_state_publisher'
        ),
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])

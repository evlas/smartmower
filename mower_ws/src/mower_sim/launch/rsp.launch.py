from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use sim time if true')

    declare_urdf = DeclareLaunchArgument(
            name='urdf',
            default_value=PathJoinSubstitution([FindPackageShare('mower_description'), 'urdf', 'mower_gz.urdf.xacro']),
            description='Path to the robot description file')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', urdf]),
                value_type=str
            )
        }]
    )

    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        robot_state_publisher
    ])

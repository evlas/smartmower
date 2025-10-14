import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_sim = get_package_share_directory('mower_sim')

    # Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.2')

    # World
    default_world = os.path.join(pkg_sim, 'worlds', 'obstacles.world')
    world = LaunchConfiguration('world', default=default_world)

    # Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world]
        }.items()
    )

    # Gazebo client
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g'
        }.items()
    )

    # RSP for mower_description using mower_gz.urdf.xacro
    mower_description_share = FindPackageShare('mower_description')
    urdf_path = PathJoinSubstitution([mower_description_share, 'urdf', 'mower_gz.urdf.xacro'])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': urdf_path
        }.items()
    )

    # Spawn from robot_description
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'mower',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Bridge
    bridge_params = os.path.join(pkg_sim, 'config', 'gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Path al world SDF'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.2'),
        gz_server,
        gz_client,
        rsp,
        bridge,
        spawn,
    ])

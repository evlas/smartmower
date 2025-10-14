import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('lawn_mower_robot')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.2')

    # Resolve world file from this package and expose as launch argument
    # Default to obstacles.world as requested
    default_world = os.path.join(pkg_dir, 'worlds', 'obstacles.world')
    world = LaunchConfiguration('world')

    # Gazebo server (run + server) with explicit world path
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world]
        }.items()
    )
    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-g'
        }.items()
    )

    # Include RSP launch (mirrors diff_drive_robot)
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': urdf_path
        }.items()
    )

    # Spawn robot in Gazebo (from robot_description topic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lawn_mower_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Bridge using config file (mirrors diff_drive_robot)
    bridge_params = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )


    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'lawn_mower.rviz')]
    )

    # Alias TF: some camera messages use frame_id 'lawn_mower_robot/base_link/camera'
    # while the TF tree publishes 'camera_frame'. Provide an identity static TF so RViz can resolve it.
    camera_frame_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_alias_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_frame', 'lawn_mower_robot/base_link/camera'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=default_world,
            description='Full path to the world SDF file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.2',
            description='Initial z position of the robot'
        ),
        gazebo_server,
        gazebo_client,
        rsp,
        ros_gz_bridge,
        spawn_robot,
        camera_frame_alias_tf,
        rviz
    ])

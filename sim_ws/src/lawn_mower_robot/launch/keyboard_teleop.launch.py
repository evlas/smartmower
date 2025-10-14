from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Allow remapping of the output cmd_vel topic if desired
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    use_terminal = LaunchConfiguration('use_terminal', default='true')

    # Build argument '/cmd_vel:=<topic>' safely using a substitution
    remap_arg = PythonExpression(["'/cmd_vel:=' + ", cmd_vel_topic])

    teleop = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '-r', remap_arg
        ],
        shell=False,
        output='screen',
        name='teleop_twist_keyboard',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Target cmd_vel topic to publish'
        ),
        DeclareLaunchArgument(
            'use_terminal',
            default_value='true',
            description='Launch teleop inside a new gnome-terminal to ensure a real TTY'
        ),
        teleop,
    ])

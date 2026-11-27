from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_config = PathJoinSubstitution(
        [FindPackageShare('hitech_bindings'), 'config', 'serial_config.yaml']
    )
    motor_config = PathJoinSubstitution(
        [FindPackageShare('hitech_bindings'), 'config', 'hitech_driver.yaml']
    )
    transport = LaunchConfiguration('transport')

    return LaunchDescription([
        DeclareLaunchArgument(
            'transport', default_value='udp',
            choices=['udp', 'serial'],
            description='udp: Feather on the Navigator via its UDP bridge; '
                        'serial: Feather plugged into this machine over USB'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('hitech_bindings'),
                                      'launch', 'udp_bridge.launch.py'])
            ),
            condition=IfCondition(
                PythonExpression(["'", transport, "' == 'udp'"])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('serial_driver'), 'launch',
                                      'serial_driver_bridge_node.launch.py']
                                     ),
            ),
            launch_arguments={'params_file': serial_config}.items(),
            condition=IfCondition(
                PythonExpression(["'", transport, "' == 'serial'"])),
        ),
        Node(
            package='hitech_bindings',
            executable='hitech_driver',
            parameters=[
                motor_config
            ]
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_inputs'
        ),

        Node(
            package='hitech_bindings',
            executable='joyControl.py',
            name='joy_bindings'
        )
    ])

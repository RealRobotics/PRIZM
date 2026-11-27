"""Hitec driver talking to the CAN Feather over the Navigator UDP bridge."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the UDP bridge nodes and hitech_driver."""
    motor_config = PathJoinSubstitution(
        [FindPackageShare('hitech_bindings'), 'config', 'hitech_driver.yaml']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('hitech_bindings'),
                                      'launch', 'udp_bridge.launch.py'])
            ),
        ),
        Node(
            package='hitech_bindings',
            executable='hitech_driver',
            parameters=[motor_config]
        )
    ])

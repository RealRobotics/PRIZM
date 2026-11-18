from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    serial_config = PathJoinSubstitution(
        [FindPackageShare("hitech_bindings"), "config", "serial_config.yaml"]
    )
    motor_config = PathJoinSubstitution(
        [FindPackageShare("hitech_bindings"), "config", "hitech_driver.yaml"]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("serial_driver"), "launch",
                                      "serial_driver_bridge_node.launch.py"]
                                     ),
            ),
            launch_arguments={"params_file": serial_config}.items(),
        ),
        Node(
            package="hitech_bindings",
            executable="hitech_driver",
            parameters=[
                motor_config
            ]
        )
    ])

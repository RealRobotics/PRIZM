from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="serial_driver", 
            executable="serial_bridge",
            name="serial_bridge",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('hitech_bindings'), 'config', 'serial_config.yaml'
                ])
            ]
        )
    ])
from sys import executable

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="game_controller_node",
            name="game_controller_node",
            parameters=[{
                "autorepeat_rate": 0.0}]
        ),

        Node(
            package="prizm_dynamixel",
            executable="calibrateServo.py",
            name="calibrateServo",
            parameters=[{"servo_id": 0}]
        )
    ])

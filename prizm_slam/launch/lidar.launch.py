from sys import executable
import numpy as np
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # rplidar startup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_c1_launch.py'
                ])
            ]),
            launch_arguments={
                'serial_port': '/dev/rplidar',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('icm20948_ros2'),
                    'launch',
                    'icm20948.launch.py'
                ])
            ])
        ),
        Node(
            package="prizm_slam",
            executable="scan_to_pc",
            name="scan_to_pc"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('prizm_slam'),
                    'launch',
                    'frames.launch.py'
                ])
            ])
        ),

    ])

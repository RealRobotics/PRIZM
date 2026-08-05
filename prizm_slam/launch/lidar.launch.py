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
                'use_provided_red': 'True'
            }.items()
        ),
        Node(
            package="prizm_slam",
            executable="scan_to_pc",
            name="scan_to_pc"
        )
                
    ])

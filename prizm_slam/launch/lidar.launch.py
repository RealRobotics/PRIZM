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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll',
                '0', '--frame-id', 'world', '--child-frame-id', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.08',
                '--yaw', '0', '--pitch', '0', '--roll',
                '3.1415', '--frame-id', 'map', '--child-frame-id', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '-0.01', '--y', '0.0', '--z', '0.03',
                '--yaw', '3.14', '--pitch', '1.57', '--roll',
                '3.14', '--frame-id', 'map', '--child-frame-id', 'imu']
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--yaw', '0', '--pitch', '0', '--roll',
        #         '0', '--frame-id', 'laser', '--child-frame-id', 'imu']
        # ),

    ])

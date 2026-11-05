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

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll',
                '0', '--frame-id', 'world', '--child-frame-id', 'base_link']
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
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll',
                '0', '--frame-id', 'odom', '--child-frame-id', 'map']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--yaw', '0', '--pitch', '0', '--roll',
        #         '0', '--frame-id', 'world', '--child-frame-id', 'odom']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.065', '--y', '0', '--z', '0.07',
                '--yaw', '0', '--pitch', '0', '--roll',
                '3.1415', '--frame-id', 'base_link', '--child-frame-id', 'laser']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--yaw', '0', '--pitch', '0', '--roll',
        #         '0', '--frame-id', 'laser', '--child-frame-id', 'odom']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[
        #         '--x', '-0.01', '--y', '0.0', '--z', '0.03',
        #         '--roll', '1.57', '--pitch', '0', '--yaw',
        #         '-1.57', '--frame-id', 'map', '--child-frame-id', 'imu']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0.0', '--z', '0.045',
                '--roll', '0', '--pitch', '0', '--yaw',
                '0', '--frame-id', 'base_link', '--child-frame-id', 'imu']
        ),


    ])

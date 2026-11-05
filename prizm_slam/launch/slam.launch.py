from sys import executable

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([


        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('prizm_slam'),
                            'launch',
                            'frames.launch.py'
                        ])
                    ])
                ),
       IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rko_slam'),
                    'launch',
                    'odometry_and_slam.launch.py'
                ])
            ]),
            launch_arguments={
                'rko_lio_config_file': 'src/PRIZM/prizm_slam/config/prizm_lio.yaml',
                'config_file' : 'src/PRIZM/prizm_slam/config/slam.yaml',
                'rviz': 'true',
                'rviz_config_file': 'src/PRIZM/prizm_slam/config/slam.rviz',
                'mode': 'online',
            }.items()
        ),
        ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', 'src/PRIZM/bags/2026_09_24_StraightPipe_1/', '--exclude-topics', '/tf_static'],
                    output='screen'
                )
    ])

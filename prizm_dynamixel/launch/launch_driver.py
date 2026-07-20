from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="prizm_dynamixel", 
            executable="driver",
            name="driver",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('prizm_dynamixel'), 'config', 'servo_driver.yaml'
                ])
            ]
        )
    ])
"""Start the udp_driver sender/receiver for the Navigator UDP bridge."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def auto_activate(node):
    """Configure the lifecycle node once it starts, then activate it."""
    return [
        RegisterEventHandler(OnProcessStart(
            target_action=node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )),
    ]


def generate_launch_description():
    """Launch udp_sender_node and udp_receiver_node with udp_config.yaml."""
    udp_config = PathJoinSubstitution(
        [FindPackageShare('hitech_bindings'), 'config', 'udp_config.yaml']
    )

    sender = LifecycleNode(
        package='udp_driver',
        executable='udp_sender_node_exe',
        name='udp_sender_node',
        namespace='',
        parameters=[udp_config],
        output='screen',
    )
    receiver = LifecycleNode(
        package='udp_driver',
        executable='udp_receiver_node_exe',
        name='udp_receiver_node',
        namespace='',
        parameters=[udp_config],
        output='screen',
    )

    return LaunchDescription(
        [sender, receiver] + auto_activate(sender) + auto_activate(receiver)
    )

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_navigation',
            executable='line_perception_node',
            name='line_perception_node',
            output='screen'
        ),
        Node(
            package='line_navigation',
            executable='line_controller_node',
            name='line_controller_node',
            output='screen'
        ),
        Node(
            package='line_navigation',
            executable='branch_decision_node',
            name='branch_decision_node',
            output='screen'
        ),
        Node(
            package='line_navigation',
            executable='cmd_arbiter_node',
            name='cmd_arbiter_node',
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wheelchair_navigation",
            executable="nav_avoidance_node.py",
            name="nav_avoidance_node",
            output="screen",
        ),
    ])
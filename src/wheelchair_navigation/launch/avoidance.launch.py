from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # false: 자동 회피/신호등 정지 없이 수동 주행만
        DeclareLaunchArgument(
            "auto_mode",
            default_value="true",
        ),

        Node(
            package="wheelchair_navigation",
            executable="nav_avoidance_node.py",
            name="nav_avoidance_node",
            output="screen",
            parameters=[{
                "auto_mode": ParameterValue(
                    LaunchConfiguration("auto_mode"),
                    value_type=bool,
                ),
            }],
        ),
    ])
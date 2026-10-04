#!/usr/bin/env python3
"""
SWAN kit wheelchair, full stack.

Same layout as swan_bringup/swan.launch.py, but every piece that was changed
for the SWAN kit comes from this package, so the original packages stay as
they were:
  ros2 launch swan_kit swan_kit.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    kit_share = get_package_share_directory("swan_kit")
    navigation_share = get_package_share_directory("wheelchair_navigation")

    # Gazebo + bridges + drive/stow controller + sensor TFs
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kit_share, "launch", "simulation.launch.py")
        )
    )

    # LiDAR self-filter tuned for the SWAN kit geometry
    scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[os.path.join(kit_share, "config", "scan_filter.yaml")],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    # Nav2 servers from wheelchair_navigation, with the SWAN kit parameters
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(kit_share, "config", "nav2_params.yaml"),
        }.items(),
    )

    avoidance_node = Node(
        package="swan_kit",
        executable="swan_kit_avoidance_node.py",
        name="nav_avoidance_node",
        output="screen",
    )

    situation_node = Node(
        package="swan_kit",
        executable="swan_kit_situation_node.py",
        name="situation_node",
        output="screen",
    )

    yolo_node = Node(
        package="wheelchair_vision",
        executable="yolo_node",
        output="screen",
    )

    raw_camera_view = Node(
        package="image_tools",
        executable="showimage",
        name="raw_camera_view",
        output="screen",
        remappings=[("image", "/camera")],
    )

    yolo_camera_view = Node(
        package="image_tools",
        executable="showimage",
        name="yolo_camera_view",
        output="screen",
        remappings=[("image", "/yolo/image_raw")],
    )

    return LaunchDescription([
        simulation_launch,
        scan_filter,
        navigation_launch,
        avoidance_node,
        situation_node,
        yolo_node,
        raw_camera_view,
        yolo_camera_view,
    ])

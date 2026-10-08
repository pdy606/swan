#!/usr/bin/env python3
"""
SWAN kit wheelchair, full stack.

Same as swan_bringup/swan.launch.py, except that the simulation (wheelchair
model, world, drive/stow controller, sensor bridges) comes from this package.
Avoidance, situation, Nav2 and the LiDAR filter are the original nodes and
settings from wheelchair_navigation / swan_pipeline.

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

    def include(share, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", name)
            )
        )

    # SWAN kit: Gazebo + bridges + drive/stow controller + sensor TFs
    simulation_launch = include(kit_share, "simulation.launch.py")

    # Original navigation stack
    scan_filter_launch = include(navigation_share, "scan_filter.launch.py")
    navigation_launch = include(navigation_share, "navigation.launch.py")
    avoidance_launch = include(navigation_share, "avoidance.launch.py")

    situation_node = Node(
        package="swan_pipeline",
        executable="situation_node",
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
        scan_filter_launch,
        navigation_launch,
        avoidance_launch,
        situation_node,
        yolo_node,
        raw_camera_view,
        yolo_camera_view,
    ])

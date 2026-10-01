#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory(
        "wheelchair_gazebo"
    )

    navigation_share = get_package_share_directory(
        "wheelchair_navigation"
    )

    # --------------------------------------------------------------
    # Gazebo simulation
    # --------------------------------------------------------------

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "simulation.launch.py",
            )
        )
    )

    # --------------------------------------------------------------
    # LiDAR filter
    # 원래 직접 실행하던:
    # ros2 launch wheelchair_navigation scan_filter.launch.py
    # --------------------------------------------------------------

    scan_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "scan_filter.launch.py",
            )
        )
    )

    # --------------------------------------------------------------
    # Nav2
    # --------------------------------------------------------------

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "navigation.launch.py",
            )
        )
    )

    # --------------------------------------------------------------
    # Navigation supervisor / avoidance
    # --------------------------------------------------------------

    avoidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                navigation_share,
                "launch",
                "avoidance.launch.py",
            )
        ),
        launch_arguments={
            "auto_mode": LaunchConfiguration("auto_mode"),
        }.items(),
    )

    # --------------------------------------------------------------
    # Situation pipeline
    # --------------------------------------------------------------

    situation_node = Node(
        package="swan_pipeline",
        executable="situation_node",
        output="screen",
    )

    # --------------------------------------------------------------
    # YOLO
    # --------------------------------------------------------------

    yolo_node = Node(
        package="wheelchair_vision",
        executable="yolo_node",
        output="screen",
    )

    # --------------------------------------------------------------
    # Gazebo raw camera
    # --------------------------------------------------------------

    raw_camera_view = Node(
        package="image_tools",
        executable="showimage",
        name="raw_camera_view",
        output="screen",
        remappings=[
            ("image", "/camera"),
        ],
    )

    # --------------------------------------------------------------
    # YOLO result camera
    # --------------------------------------------------------------

    yolo_camera_view = Node(
        package="image_tools",
        executable="showimage",
        name="yolo_camera_view",
        output="screen",
        remappings=[
            ("image", "/yolo/image_raw"),
        ],
    )

    return LaunchDescription([
        # ros2 launch swan_bringup swan.launch.py auto_mode:=false
        # -> 자동 회피/신호등 정지 끄고 수동 주행만
        DeclareLaunchArgument(
            "auto_mode",
            default_value="true",
        ),

        simulation_launch,
        scan_filter_launch,
        navigation_launch,
        avoidance_launch,
        situation_node,
        yolo_node,
        raw_camera_view,
        yolo_camera_view,
    ])
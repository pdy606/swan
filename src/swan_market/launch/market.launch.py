"""Backward-compatible shortcut to the team's shared world launcher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    common = Path(get_package_share_directory('wheelchair_gazebo')) / 'launch/simulation.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(common)),
        launch_arguments={'world':'market_shopping.sdf', 'world_package':'swan_market'}.items())])

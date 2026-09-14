"""Optional market traffic; common launcher supplies the selected world name."""
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('swan_market'))
    return LaunchDescription([
        DeclareLaunchArgument('moving_traffic', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='swan_market'),
        ExecuteProcess(cmd=[sys.executable, str(share/'scripts/animate_crosswalk.py'),
            '--layout', str(share/'config/market_layout.json'), '--world', LaunchConfiguration('world_name')],
            condition=IfCondition(LaunchConfiguration('moving_traffic')), output='screen'),
    ])

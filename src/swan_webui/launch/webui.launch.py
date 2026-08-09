"""폰 웹 UI: rosbridge(WebSocket 9090) + 정적 웹서버(8000) 동시 실행."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('swan_webui'), 'web')
    rosbridge = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        # ROS2 토픽 ↔ WebSocket 다리 (XML 런치라 XMLLaunchDescriptionSource 사용)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(rosbridge, 'launch', 'rosbridge_websocket_launch.xml')),
        ),
        # 정적 페이지 서빙 (폰이 접속할 곳)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000', '--directory', web_dir],
            output='screen'),
    ])

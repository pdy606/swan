#!/usr/bin/env bash
# Start the recording scene or its keyboard driver in the existing UTM workspace.
set -e

usage() {
  cat <<'HELP'
사용법: start_recording.sh [apartment|market|city] [추가 launch 인자...]
        start_recording.sh drive

기본 출발 구역: apartment
기본 작업공간: ~/world_integration_ws
다른 작업공간은 SWAN_INTEGRATION_WS 환경변수로 지정합니다.

예: start_recording.sh market
    start_recording.sh city show_camera:=false
    start_recording.sh drive  # 별도 터미널에서 키보드 운전
HELP
}

mode=scene
case "${1:-apartment}" in
  -h|--help) usage; exit 0 ;;
  drive) mode=drive ;;
  apartment|market|city) zone="${1:-apartment}" ;;
  *) printf '알 수 없는 출발 구역: %s\n' "$1" >&2; usage >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi

recording_workspace="${SWAN_INTEGRATION_WS:-$HOME/world_integration_ws}"
ros_setup=/opt/ros/jazzy/setup.bash
workspace_setup="$recording_workspace/install/setup.bash"

if [[ ! -r "$ros_setup" ]]; then
  printf 'Ubuntu/UTM의 ROS Jazzy 환경에서 실행해 주세요: %s\n' "$ros_setup" >&2
  exit 1
fi
if [[ ! -r "$workspace_setup" ]]; then
  printf '빌드된 작업공간을 찾을 수 없습니다: %s\n' "$workspace_setup" >&2
  printf 'SWAN_INTEGRATION_WS에 world_integration_ws 경로를 지정해 주세요.\n' >&2
  exit 1
fi

# Do not use nounset: ROS setup scripts can reference unset environment variables.
source "$ros_setup"
source "$workspace_setup"

if [[ "$mode" == drive ]]; then
  printf '이 터미널에서 운전합니다: i 전진, , 후진, j/l 회전, k 정지 (종료: Ctrl+C)\n'
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/model/wheelchair/cmd_vel \
    -p speed:=0.15 -p turn:=0.3 -p repeat_rate:=10.0 -p key_timeout:=0.2 "$@"
fi

printf '촬영 장면을 엽니다. 출발 구역: %s (종료: Ctrl+C)\n' "$zone"
exec ros2 launch swan_world_integration recording.launch.py \
  "start_zone:=$zone" software_rendering:=true "$@"

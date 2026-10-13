# 통합 월드: swan_town.sdf

월드 파일은 `src/wheelchair_gazebo/worlds/swan_town.sdf` 하나입니다.
아파트·주차장, 도시 인도, 시장과 연결로, 휠체어 1대 및 카메라·LiDAR가 내장되어 있습니다. 외부 mesh, model URI, 다른 SDF include가 없습니다.

```bash
source /opt/ros/jazzy/setup.bash
gz sim -r src/wheelchair_gazebo/worlds/swan_town.sdf
```

Gazebo Harmonic 기준입니다. SDF만 실행하면 시장 사람·오토바이·신호는 초기 상태에 머뭅니다. 이동·신호 점멸은 `feature/world-integration`의 `swan_world_integration` 패키지에 보존된 원본 Python 제어기로 실행합니다. ROS 브리지와 주행 제어기는 SDF와 별도입니다.

기존 `wheelchair_world.sdf`, `swan_kit_world.sdf`, 팀원 모델·노드·공통 launch는 변경하지 않았습니다. `ros2 launch swan_bringup swan.launch.py`는 기존 월드를 계속 사용합니다. 이 launch에는 world 선택 인자가 없으므로 통합 실행 테스트는 별도 작업공간 복사본에서만 기본 월드 자산을 이 파일로 교체해 진행합니다. 원본 작업공간을 덮어쓰지 않습니다.

내장 휠체어는 일반 조향 모델이며 접기 제어는 없습니다. 접기 기능은 integration 브랜치의 별도 `swan_kit` 모델에 있습니다.

원본 출처:
- 아파트·주차장: dayeong `a23e859` (환경 모델 7개)
- 도시: yeonho `7adea9e` (72개)
- 시장: seunghyun `011fa32` (251개)
- 내장 휠체어: dayeong `a23e859`

원본 환경 모델 내부와 휠체어 내부를 보존하고 구역 배치만 변환했습니다. 연결로 2개와 도시 진입 경사로 1개를 추가했습니다. 총 334개 모델(환경 330 + 연결 3 + 휠체어 1)입니다. 통합 소스는 `feature/world-integration`의 `62452ad`와 동일합니다.

2026-10-13: XML 및 외부 참조 검사, 원본 보존 검사, UTM Gazebo 화면·카메라·LiDAR·실제 바퀴 주행을 확인했습니다. 안전 제어 성능이나 무충돌을 인증한 결과는 아닙니다.

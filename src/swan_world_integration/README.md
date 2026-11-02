# SWAN 통합 월드 — 재현용 촬영 패키지

2026-11-02 제출 브랜치: **`seunghyun`**. 팀 전체 코드는 `dayeong da3fd917` 기준이다. 이 패키지는 기존 통합 월드와 10월 검증 조건을 보존하는 별도 실행 방식이며, 공통 `simulation.launch.py`나 최신 휠체어 모델을 교체하지 않는다.

## 구성과 리소스

**[worlds/swan_town.sdf](worlds/swan_town.sdf)** 한 개에 아파트·주차장, 도시, 시장, 연결로, 기존 휠체어·카메라·LiDAR가 들어 있다. 별도 로봇 SDF, 외부 mesh·텍스처·모델 URI와 다운로드 없이 실행한다. Gazebo Harmonic 시스템 플러그인은 설치되어 있어야 한다.

| 원본 | 고정 커밋 | 환경 모델 |
|---|---|---:|
| 아파트·주차장 | `a23e859` | 7 |
| 도시 | `7adea9e` | 72 |
| 시장 | `011fa32` | 251 |

환경 모델 330개 내부는 보존하고 구역 전체에 이동·회전만 적용했다. 연결로·경사로 3개를 추가하고 `a23e859` 휠체어 한 대를 내장했다. 출처·해시·좌표는 [integration.json](config/integration.json), 스크립트 출처는 [vendor_sources.json](config/vendor_sources.json)에 있다. 구동 제어기와 시장 교통 스크립트는 원본 바이트를 유지한다. 월드 SHA-256:

```text
d7b9b54f720abb059779acf650dffee4bb46076c64b86374dccbbc60cc2716f2
```

기존 `integration`의 `src/wheelchair_gazebo/worlds/swan_town.sdf`는 다시 복사하지 않았다. 여기서는 자체 SDF와 촬영 설정을 사용한다.

## 빌드·실행·종료

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 작업공간 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swan_world_integration
source install/setup.bash
ros2 launch swan_world_integration recording.launch.py start_zone:=apartment moving_traffic:=false
```

출발 구역은 `apartment` / `market` / `city`다. 다른 구역은 임시 복사본의 로봇 시작점만 바꾸며 저장된 SDF는 유지한다. `show_camera:=true`가 기본이다. 시장 이동 교통·신호 전환은 `moving_traffic:=true`, 렌더링 문제가 있으면 `software_rendering:=true`를 추가한다. 이 명령은 자동 경로 주행기가 아니다.

운전용 별도 터미널:

```bash
source /opt/ros/jazzy/setup.bash
source ~/swan_seunghyun_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/model/wheelchair/cmd_vel \
  -p speed:=0.15 -p turn:=0.3 -p repeat_rate:=10.0 -p key_timeout:=0.2
```

`i` 전진 / `,` 후진 / `j`·`l` 회전 / `k` 정지. `k`로 정지한 뒤 운전 터미널 `Ctrl+C`, 월드 터미널 `Ctrl+C` 순서로 종료하고 프롬프트 복귀를 확인한다. 다른 폴더를 썼다면 source 경로를 바꾼다. [설치·YOLO 별도 실행·종료 안내](../../docs/integration/LINUX_RUN_GUIDE.txt)를 참고한다.

SDF와 센서만 열려면 작업공간 루트에서 `gz sim -r src/swan_world_integration/worlds/swan_town.sdf`를 사용한다. ROS 브리지·교통 제어는 이 단독 명령에 포함되지 않는다. 기존 `start_recording.sh`도 유지하며 `SWAN_INTEGRATION_WS=~/swan_seunghyun_ws`로 작업공간을 지정할 수 있다.

## 기존 실행 근거

- **10/24:** 아파트 경사로·주차장 → 시장 끝 → 같은 골목을 후진해 도시 분기점 → 도시 북쪽 보행로, 약 **180.635m**, 4/4 구간. 시뮬레이션 457.165초 / 실제 1,233.814초. 중간 위치 강제 이동과 드라이버 재시작 없음. 설치된 월드·제어기 해시가 검증 대상과 일치했다.
- **10/13:** GUI·카메라·LiDAR·저속 이동·정지·키보드 운전·시장 교통·신호 점멸 확인. 당시 [실측 로그](../../docs/integration/recording_check_20261013.json)를 보존한다.
- **11/2:** 기존 영상·로그·해시 검토와 정적 검사 20개 통과. 이번 제출에서 VM·Gazebo·YOLO를 실행하지 않았다.

10/24는 **정지 교통에서 고정 경로 속도 명령을 사용한 연결 동선 검증**이다. 시장→도시는 후진으로 돌아왔으며 골목 U턴 시험이 아니다. 접촉 이벤트를 수집하지 않아 무충돌이라고 판정하지 않는다. 최신 Vision 거리 측정·Gap 계산·신호 판단·Nav2 회피·API 운전자·이동 장애물 대응·안전 기능 비교는 별도 검증이다. 조건과 영상·로그는 [검증 요약](../../docs/integration/CONNECTED_ROUTE_CHECK_20261024.txt)에 있다.

## 알려진 제한과 남은 확인

- 재현용 로봇·제어기는 10월 버전이다. 최신 팀 전체 실행은 `ros2 launch swan_bringup swan.launch.py`를 사용하며 촬영 런처와 동시에 켜지 않는다.
- 아파트 입구는 고정 개방 상태다. 자동문 개폐는 재현하지 않는다. 원본의 약 3.03cm 단차와 일부 틈을 유지했다.
- 도시 원본 보행자·신호등은 정적이다. 이동 교통·신호 전환은 시장 구역 기능이다.
- 원본 `building_structure` 아래 `lobby_light` SDF 경고는 기존 실행에서 관찰했으며 원본을 보존했다.
- 팀원 PC 새 설치·빌드·종료, 최신 전체 스택 실행·인식 정확도는 확인이 남아 있다. 표본 영상 재생 길이를 실제 실행 시간으로 해석하지 않는다.

## 시뮬레이터 없이 검사

작업공간 루트에서:

```bash
python3 -m unittest discover -s src/swan_world_integration/test -p 'test_*.py' -v
```

환경·내장 로봇 내부 해시, 구역별 위치, 누락·중복, 외부 자산 참조, 런처 연결과 임시 복사본 정리를 검사한다. Git 원본이 있으면 실제 소스 바이트와 대조한다. 별도 저장소는 `SWAN_SOURCE_REPO=/기존/저장소/경로`로 지정한다. 원본 Git이 없으면 대조 항목만 건너뛴다. 네트워크 다운로드·VM·Gazebo를 실행하지 않는다.

# 시장 월드 실행

배포하는 맵은 `wheelchair_gazebo/worlds/market_shopping.sdf` 하나다.
SDF 1.10 형식이며, 환경 형상과 한글 간판이 파일 안에 포함돼 있다.

## 빌드와 실행

ROS2 Jazzy / Gazebo Harmonic 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash

# 시장·휠체어·센서·이동 교통·신호 전환
ros2 launch swan_market world.launch.py
# 같은 시장 실행의 단축 명령
ros2 launch swan_market market.launch.py
```

기존 Gazebo 실행을 종료한 다음 새로 실행한다.
`ros2 launch wheelchair_gazebo simulation.launch.py`도 시장 맵과 휠체어를 연다.
이 기본 런처는 `wheelchair_gazebo`만으로 실행하며, 이동 교통·신호 전환 제어기는 포함하지 않는다.

다음 옵션은 `swan_market world.launch.py`에서 사용한다.

| 옵션 | 기본값 / 동작 |
|---|---|
| `world` | `market_shopping.sdf`; 파일명, 절대 경로 또는 등록 별칭 |
| `world_package` | 파일명 검색 패키지. 생략 시 `wheelchair_gazebo` |
| `headless` | `false`; `true`이면 GUI 없이 실행 |
| `software_rendering` | `false`; UTM에서는 `true`로 llvmpipe 사용 |
| `moving_traffic` | `true`; 이동 교통과 신호 전환 실행 |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | 월드에 로봇이 없을 때 초기 위치 덮어쓰기. 단위 m / rad |

```bash
ros2 launch swan_market world.launch.py software_rendering:=true
ros2 launch swan_market world.launch.py moving_traffic:=false
```

## 다른 브랜치 맵으로 교체할 때

이 브랜치에 다른 맵의 복사본을 보관하지 않아도 외부 월드의 절대 경로를 지정할 수 있다.

```bash
ros2 launch swan_market world.launch.py world:=/absolute/path/custom.sdf spawn_x:=0 spawn_y:=-5 spawn_yaw:=1.57
```

월드는 `<sdf><world name="고유이름">...</world></sdf>` 구조여야 한다.
환경만 있는 월드에는 공통 휠체어를 한 대 생성한다. 이미 `model name="wheelchair"` 또는
`model://wheelchair` include가 있으면 중복 생성하지 않고 기존 위치를 사용한다.
이때 `spawn_*` 덮어쓰기는 오류로 처리한다.

시장 초기 위치 `(-6.2, 0, 0.03, yaw=0)`와 교통 런처는 `config/worlds.json`에 등록돼 있다.
별칭 `market`과 파일명 `market_shopping.sdf`는 같은 설정을 사용한다.
미등록 외부 월드의 기본 위치는 `(0, 0, 0.3, yaw=0)`이므로 지형에 맞게 지정한다.
외부 파일 옆에 `<파일명>.launch.json`이 있으면 초기 위치·시나리오 설정을 읽을 수 있다.

## 휠체어·센서

- 로봇 이름: `wheelchair`; 기본 프레임: `base_link`.
- 명령 입력: `/model/wheelchair/cmd_vel` (`geometry_msgs/msg/Twist`).
- 출력: `/odom`, `/tf`, `/clock`, `/scan`; 카메라가 있는 모델은 `/camera`.
- 시장 런처는 모델 SDF의 카메라·GPU LiDAR 토픽과 링크·센서 pose를 읽어 연결한다.
- YOLO 추론 노드는 별도 실행이다.
- 이동 인물과 오토바이는 스크립트로 위치를 갱신한다. 휠체어에 대한 양보·회피 기능은 없다.
- 맵 SDF만 직접 열면 초기 배치가 보이고, 이동과 신호 전환에는 시장 런처가 필요하다.

## 검사

```bash
python3 -m unittest discover -s src/swan_market/test -v
```

단일 시장 파일, 두 런처의 기본 맵·로봇 생성, 외부 월드의 로봇 중복 방지,
초기 위치·센서 연결·신호 주기를 오프라인에서 검사한다.
ROS launch 객체는 테스트 대역을 사용하므로 실제 Gazebo 실행 검증을 대체하지 않는다.
이전 실행 기록과 사진은 [검증 요약](../../docs/market/README.md)에 있다.

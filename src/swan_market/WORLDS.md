# 팀 월드 선택 실행

시장 패키지가 제공하는 `world.launch.py`에서 월드를 선택한다.
팀 공용 `wheelchair_gazebo/launch/simulation.launch.py`는 integration 원본을 유지한다. `.world`와 `.sdf` 모두 SDF XML이다.
기존 팀 월드는 SDF 1.9, 시장 월드는 SDF 1.10이다. 시장 파일 확장자는 팀 월드와 같은 `.world`를 사용한다.

## 빌드와 실행

ROS2 Jazzy / Gazebo Harmonic 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash

# integration 기본 시험 월드: 기존 실행 명령 유지
ros2 launch wheelchair_gazebo simulation.launch.py
# 시장: 보행자 2명 + 오토바이 2대 이동 포함
ros2 launch swan_market world.launch.py world:=market
# 기존 팀 월드
ros2 launch swan_market world.launch.py world:=layout_narrow_alley.world
ros2 launch swan_market world.launch.py world:=level1_basic.world
```

기존 Gazebo 실행을 종료한 다음 월드를 바꾼다. 실행 중 월드를 교체하는 기능은 아니다.
시장 단축 명령 `ros2 launch swan_market market.launch.py`도 계속 사용할 수 있다.
이 문서의 `world.launch.py`는 `swan_market` 패키지가 필요하다.
팀의 기존 기본 시험 월드 실행은 `wheelchair_gazebo simulation.launch.py`로 독립 실행할 수 있다.

| 옵션 | 기본값 / 동작 |
|---|---|
| `world` | `wheelchair_world.sdf`; 파일명, 절대 경로 또는 등록 별칭 |
| `world_package` | 파일명 검색 패키지. 생략 시 `wheelchair_gazebo`; 시장 파일도 이 패키지에 있음 |
| `headless` | `false`; `true`이면 GUI 없이 실행 |
| `software_rendering` | `false`; UTM에서는 `true`로 llvmpipe 사용 |
| `moving_traffic` | `true`; 시장 이동 교통에 적용 |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | 월드에 로봇이 없을 때 초기 위치 덮어쓰기. 단위 m / rad |

```bash
ros2 launch swan_market world.launch.py world:=market software_rendering:=true
ros2 launch swan_market world.launch.py world:=market moving_traffic:=false
ros2 launch swan_market world.launch.py world:=/absolute/path/custom.world spawn_x:=0 spawn_y:=-5 spawn_yaw:=1.57
ros2 launch swan_market world.launch.py world:=market_shopping.world
```

## 월드 추가 규칙

1. `<sdf><world name="고유이름">...</world></sdf>` 파일을 패키지의 `worlds/`에 둔다.
   시장을 포함한 모든 팀 월드는 `wheelchair_gazebo/worlds/` 한 곳에 둔다.
   기존 월드 생성기는 main과 같은 `wheelchair_gazebo/worlds/`, 시장 생성기는 `swan_market/tools/`에 둔다.
   이는 `main` 브랜치의 월드 위치와 같다.
2. 신규 월드는 환경만 담고 휠체어는 `swan_market world.launch.py`가 생성하도록 권장한다.
   기존 월드의 `model://wheelchair` include 또는 `model name="wheelchair"`는 유지해도 된다.
   이 경우 추가 생성하지 않고 월드에 적힌 초기 위치를 사용한다. `spawn_*`를 지정하면
   무시하지 않고 설명과 함께 실패한다.
3. 초기 위치가 필요하면 같은 디렉터리에 `<파일이름>.launch.json`을 둔다.
   `example.world`이면 `example.launch.json`이다. 시장의 실제 예제를 참고한다.

```json
{
  "spawn": {"x": -6.2, "y": 0, "z": 0.03, "yaw": 0},
  "scenario_launch": "launch/traffic.launch.py",
  "scenario_package": "swan_market"
}
```

`spawn`과 `scenario_launch`는 모두 선택 사항이다. 메타데이터가 없는 환경 전용 월드의
초기 위치는 `(0, 0, 0.3, 0)`이다. 각 월드의 장애물·지형에 맞춰 초기 위치를 정해야 하며,
기존 월드 9개의 기본 위치에서 충돌 없이 주행하는지는 이 변경에서 검증하지 않았다.
`scenario_launch`는 `scenario_package` 기준 경로이며, 이를 생략하면 월드 패키지를 사용한다.
해당 패키지의 `models/`도 자산 경로에 추가한다. 추가 런처는 `world_name`과 `moving_traffic` 인자를 받는다.
정적 월드는 이를 생략한다. 절대 경로의 파일도 같은 이름의 sidecar를 읽는다.
다른 패키지의 자산·시나리오를 쓸 경우 `world_package`도 지정한다.

4. 자산은 해당 패키지의 `models/`에 넣고 `model://...`로 참조한다. 공통 런처는
   월드 패키지와 휠체어 패키지의 `models/`, 월드 파일의 디렉터리를 리소스 경로에 추가하고
   기존 `GZ_SIM_RESOURCE_PATH`도 보존한다. CMake에서 `worlds`, `models`, `launch`, `config`를 설치한다.
5. 짧은 이름이 필요하면 `swan_market/config/worlds.json`에 `{ "별칭": {"package":"패키지", "world":"파일명"} }`을 추가한다.

## 휠체어·센서 계약

- 로봇 이름: `wheelchair`; 기본 프레임: `base_link`.
- 명령 입력: `/model/wheelchair/cmd_vel` (`geometry_msgs/msg/Twist`).
- 출력: `/odom`, `/tf`, `/clock`, `/scan`; 카메라가 있는 모델은 `/camera`.
- 모델 SDF의 카메라·GPU LiDAR `<topic>`을 읽어서 공통 ROS 토픽으로 연결한다.
  카메라가 없는 다른 브랜치 모델도 실행할 수 있으며 그 경우 `/camera`는 생성하지 않는다.
- LiDAR 링크·센서의 부모 기준 pose를 TF에 반영한다. 현재는 rad 단위 Euler pose를 지원한다.
- YOLO 추론 노드는 별도 실행이다. 월드 교체가 학습 모델이나 인식 성능을 바꾸지는 않는다.
- 시장 이동은 스크립트 방식이며 휠체어를 감지한 양보·회피 기능은 없다.

## 검증

```bash
python3 -m unittest discover -s src/swan_market/test -v
```

13개 오프라인 테스트로 기존 월드 9개의 선택, 월드 이름에 맞는 로봇 생성,
기존 로봇 중복 생성 방지, 시장 초기 위치·선택적 교통 실행, 모델별 센서 토픽,
잘못된 파일·초기 위치 처리, headless 설정과 로봇 링크·조인트 중복 여부를 검사했다.
ROS launch 객체는 테스트 대역을 사용하므로 실제 Gazebo 실행 검증을 대체하지 않는다.
공통 런처 개편 시점에는 VM이 꺼져 있어 실제 실행 재검증은 아직 하지 않았다.
시장 장면의 검증 요약과 사진은 [docs/market](../../docs/market/README.md)에 있다.
원시 로그·이전 주행 기록은 해당 문서에 연결된 Git 이력에서 찾을 수 있다.

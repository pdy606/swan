# SWAN 시장 시뮬레이션

## 맵 파일

**[market_shopping.sdf](src/wheelchair_gazebo/worlds/market_shopping.sdf)** 하나가 시장 월드다.

```text
src/wheelchair_gazebo/worlds/
└── market_shopping.sdf
```

시장 환경·가게·한글 간판·횡단보도·인물·오토바이·보행신호 형상이 이 SDF에 들어 있다.
외부 메시·텍스처 다운로드 없이 맵 파일만 Gazebo에서 열 수 있다.
기존 시험 월드 10개와 해당 생성 스크립트 2개는 제거했다. 이전 파일은 Git 이력에서 복원할 수 있다.

```bash
gz sim -r src/wheelchair_gazebo/worlds/market_shopping.sdf
```

이 명령은 환경과 초기 배치를 연다. 휠체어·센서 연결·교통 이동·신호 전환은 아래 ROS 런처를 사용한다.
`models/wheelchair/model.sdf`는 런처가 생성하는 로봇 정의이고, `CMakeLists.txt`와 `package.xml`은 ROS 패키지 빌드에 필요하다.

## 휠체어와 이동 교통 실행

ROS2 Jazzy / Gazebo Harmonic 환경의 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash
ros2 launch swan_market world.launch.py
```

기본 맵은 시장이다. `world:=market` 또는 `ros2 launch swan_market market.launch.py`도 사용할 수 있다.
UTM에서는 `software_rendering:=true`를 추가한다. 기존 Gazebo 실행은 종료한 뒤 새로 실행한다.

## 관련 문서

- [실행 옵션·월드 교체](src/swan_market/WORLDS.md)
- [시장 구성·센서 화면·맵 수정](src/swan_market/README.md)
- [이전 검증 요약·장면 사진](docs/market/README.md)
- [판단 파이프라인](src/swan_pipeline/README.md)

## integration과의 관계

`integration`의 `ff1df55`를 병합한 구조를 유지한다. 이번 정리로 이 브랜치의 `worlds/`에는 시장 SDF만 남겼고,
`wheelchair_gazebo/launch/simulation.launch.py`의 기본 맵과 로봇 생성도 시장 기준으로 변경했다.
`integration` 브랜치 자체와 다른 체크아웃은 수정하지 않았다.

`wheelchair_navigation`, `wheelchair_vision`, `swan_bringup`의 기존 구성을 사용할 수 있다.
기본 `situation_node`는 integration 버전이며, 기존 C 전용 판단은 `situation_c_node`에 보존돼 있다.
기본 통합 실행과 기존 3노드 판단 런치를 동시에 실행하지 않는다.

```bash
colcon build --packages-up-to swan_bringup --symlink-install
source install/setup.bash
ros2 launch swan_bringup swan.launch.py
```

이 통합 명령도 시장 맵과 휠체어를 연다. 이동 교통·신호 전환 제어기는 자동으로 포함하지 않는다.
전체 교통 시나리오는 위 시장 런처가 담당하며, 두 명령으로 Gazebo를 중복 실행하지 않는다.
이 통합 조합의 실제 Gazebo·Nav2·YOLO 실행 검증은 별도다.

## 오프라인 검사

```bash
python3 -m unittest discover -s src/swan_market/test -v
```

단일 맵 구성, 기본 런처의 맵·로봇 경로, 초기 위치, 센서 연결, 신호 주기를 검사한다.
실제 Gazebo 주행 실험은 실행하지 않는다. 원시 검증 기록은 Git 이력에 보관하며 새 `preview/` 출력은 커밋하지 않는다.

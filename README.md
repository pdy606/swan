# SWAN 시뮬레이션

## 월드 파일 바로 찾기

**우리 시장 맵 → [market_shopping.world](src/wheelchair_gazebo/worlds/market_shopping.world)**

모든 월드 파일은 **`src/wheelchair_gazebo/worlds/` 한 폴더**에 있다.
시장 월드도 다른 브랜치의 팀 월드와 같은 경로·`.world` 확장자로 맞췄다.

| 월드 | 실제 파일 바로가기 | 실행 시 `world:=` 값 |
|---|---|---|
| 시장·횡단보도·이동 교통 | [market_shopping.world](src/wheelchair_gazebo/worlds/market_shopping.world) | `market` |
| integration 기본 시험 월드 | [wheelchair_world.sdf](src/wheelchair_gazebo/worlds/wheelchair_world.sdf) | `wheelchair_world.sdf` |
| 좁은 골목 | [layout_narrow_alley.world](src/wheelchair_gazebo/worlds/layout_narrow_alley.world) | `layout_narrow_alley.world` |
| 직선 통로 | [layout_straight.world](src/wheelchair_gazebo/worlds/layout_straight.world) | `layout_straight.world` |
| T자 교차로 | [layout_t_junction.world](src/wheelchair_gazebo/worlds/layout_t_junction.world) | `layout_t_junction.world` |
| 넓은 광장 | [layout_open_plaza.world](src/wheelchair_gazebo/worlds/layout_open_plaza.world) | `layout_open_plaza.world` |
| S자 통로 | [layout_s_curve.world](src/wheelchair_gazebo/worlds/layout_s_curve.world) | `layout_s_curve.world` |
| 1단계 기본 | [level1_basic.world](src/wheelchair_gazebo/worlds/level1_basic.world) | `level1_basic.world` |
| 2단계 표준 | [level2_standard.world](src/wheelchair_gazebo/worlds/level2_standard.world) | `level2_standard.world` |
| 3단계 고난도 | [level3_challenge.world](src/wheelchair_gazebo/worlds/level3_challenge.world) | `level3_challenge.world` |
| 4단계 평가 | [level4_unseen_eval.world](src/wheelchair_gazebo/worlds/level4_unseen_eval.world) | `level4_unseen_eval.world` |

`worlds/`에는 환경 맵이 있고, `models/`의 `model.sdf`는 휠체어·자산 정의다.
월드 파일은 원래 위치에 한 벌만 유지한다.

## 시작

ROS2 Jazzy / Gazebo Harmonic 환경의 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash
ros2 launch swan_market world.launch.py world:=market
```

`world:=layout_narrow_alley.world` 또는 `world:=level1_basic.world`처럼 이름을 바꿔 다른 월드를 선택한다.
UTM에서는 `software_rendering:=true`를 추가한다. 기존 실행을 종료한 뒤 새 월드를 실행한다.

## 필요한 곳만 보기

- [월드 선택·추가 규칙](src/swan_market/WORLDS.md)
- [시장 구성·센서 화면·맵 수정](src/swan_market/README.md)
- [검증 요약·실제 장면 사진](docs/market/README.md)
- [기존 판단 파이프라인](src/swan_pipeline/README.md)



원시 검증 기록은 Git 이력에 보관하며 새 `preview/` 출력은 커밋하지 않는다.

## integration 기준 구조

`integration`의 `ff1df55`를 seunghyun에 병합했다. 기본 Gazebo 런처·휠체어 모델·시험 월드,
`swan_bringup`, `wheelchair_navigation`, `wheelchair_vision`의 기존 파일은 integration과 동일하다.
그 위에 시장 패키지와 추가 월드 파일을 유지한다.

```text
src/
  swan_bringup/          integration 통합 실행
  wheelchair_gazebo/    integration 원본 + worlds/시장·팀 월드
  wheelchair_navigation/ integration 내비게이션·회피
  wheelchair_vision/    integration YOLO 노드·모델
  swan_interfaces/      VisionStatus + 기존 Detection/DriveTarget 메시지
  swan_pipeline/        integration 기본 판단 + 별도 보존한 기존 3노드 구성
  swan_market/          시장 선택·이동 교통·자산·검사
  swan_webui/           기존 웹 화면
```

- 기본 `situation_node`는 integration 버전이다.
- 기존 C 전용 판단은 `situation_c_node`로 보존하고, 기존 `swan_integration.launch.py`에서만 선택한다.
- `VisionStatus`, `Detection`, `DetectionArray`, `DriveTarget` 메시지 4종을 모두 빌드한다.
- 기본 통합 실행과 기존 3노드 판단 런치를 동시에 실행하지 않는다. 같은 판단 토픽에 발행한다.
- `integration` 브랜치 자체는 변경하지 않았다. 이 병합은 seunghyun에 반영했다.

통합 기능을 사용할 때는 해당 패키지까지 빌드한 뒤 팀 명령으로 실행한다.
이 명령은 integration의 기본 시험 월드를 사용하며, 시장 장면 명령과 동시에 실행하지 않는다.

```bash
colcon build --packages-up-to swan_bringup --symlink-install
source install/setup.bash
ros2 launch swan_bringup swan.launch.py
```

시장 장면은 위의 `ros2 launch swan_market world.launch.py world:=market` 또는
`ros2 launch swan_market market.launch.py`로 실행한다.
시장 파일 경로는 계속 `src/wheelchair_gazebo/worlds/market_shopping.world`다.
오프라인 테스트 13개는 통과했으며, 이 통합 조합의 실제 Gazebo·Nav2·YOLO 실행 검증은 남아 있다.

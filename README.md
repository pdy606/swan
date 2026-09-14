# SWAN 시뮬레이션

## 월드 파일 바로 찾기

**우리 시장 맵 → [market_shopping.world](src/wheelchair_gazebo/worlds/market_shopping.world)**

모든 월드 파일은 **`src/wheelchair_gazebo/worlds/` 한 폴더**에 있다.
시장 월드도 다른 브랜치의 팀 월드와 같은 경로·`.world` 확장자로 맞췄다.

| 월드 | 실제 파일 바로가기 | 실행 시 `world:=` 값 |
|---|---|---|
| 시장·횡단보도·이동 교통 | [market_shopping.world](src/wheelchair_gazebo/worlds/market_shopping.world) | `market` |
| 기본 도시 | [wheelchair_world.sdf](src/wheelchair_gazebo/worlds/wheelchair_world.sdf) | `wheelchair_world.sdf` |
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

```text
src/
  wheelchair_gazebo/  팀 원본 Gazebo 실행·휠체어 모델
    worlds/          시장 포함 전체 월드 11개
  wheelchair_navigation/  팀 내비게이션
  swan_market/        월드 선택·시장 자산·이동 교통
  swan_interfaces/    공용 메시지
  swan_pipeline/      판단 파이프라인
  swan_webui/         웹 화면
docs/market/          검증 요약과 대표 사진 2장
```

원시 검증 기록은 Git 이력에 보관하며 새 `preview/` 출력은 커밋하지 않는다.

## 브랜치 합치기 위한 구조

- `wheelchair_gazebo`의 기존 빌드·런처·모델·도시 월드는 `dayeong`/`yeonho` 공통 파일과 동일하게 유지한다.
- 기존 월드와 생성기는 `main`의 `wheelchair_gazebo/worlds/` 위치·내용을 유지한다.
- 내비게이션은 팀 원래 경로인 `wheelchair_navigation`으로 복원했다. 공통 기준 버전(dayeong)의 코드를
  그대로 사용해, gyuwon/integration의 후속 변경이 merge될 수 있게 했다. 시장 실행에서 자동으로 구동하지 않는다.
- 시장 전용 선택 런처·별칭·검사는 `swan_market`에 둔다. 시장 월드 파일은 공용 `worlds/`에 추가한다.
- `wheelchair_gazebo simulation.launch.py world:=market` 대신
  **`swan_market world.launch.py world:=market`**을 사용한다. 공용 런처는 팀 원본으로 복원했다.
  `ros2 launch swan_market market.launch.py` 단축 명령은 계속 사용할 수 있다.

2026-09-14 원격 HEAD 기준 `git merge-tree --write-tree` 모의 merge 결과:

| 대상 | 대상 커밋 | 변경 전 충돌 메시지 | 변경 후 |
|---|---|---:|---:|
| main | `6c6ee04` | 2 | 0 |
| dayeong | `f9a0e33` | 5 | 0 |
| gyuwon | `e67f016` | 5 | 0 |
| yeonho | `7adea9e` | 5 | 0 |
| integration | `ff1df55` | 13 | 6 |

이는 Git 텍스트 병합 검사이며, 실제 merge나 통합 실행을 수행한 결과는 아니다.
`integration`에 남은 충돌은 `swan_interfaces`의 CMake/package 2개,
`swan_pipeline`의 package/setup/situation_node 3개, 기본 `wheelchair_world.sdf` 1개다.
메시지·상황판단 로직과 기본 도시 선택은 팀 통합 시 검토해야 하므로 임의로 덮어쓰지 않았다.
시장 장면의 과거 실행 기록은 모델을 팀 원본으로 맞추기 전 결과이며, 현재 조합의 실행 재검증은 남아 있다.

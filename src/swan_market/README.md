# SWAN 시장 쇼핑 맵 — 횡단보도 이동 교통 v4

담당 시나리오: **시장 도착 → 쇼핑 → 사람·오토바이를 만나는 통로 → 출구**.

시장 월드는 `swan_market` 독립 패키지로 구성되어 있다. 함께 포함된 `wheelchair_gazebo`는 휠체어 모델과 센서 실행에 사용한다.

## 구성

- 한국 전통시장 형태, 전체 바닥 46 × 24m, 주 통로 약 36m.
- 입구 간판과 양쪽 가게 8개: 과일·채소·생선·반찬·떡·분식·잡화·꽃.
- 줄무늬 차양, 한글 간판, 좌판, 낮은 상품 상자, 상품 모형.
- 시장 안 고정 인물 21명과 주정차 오토바이 3대. 횡단보도 이동 보행자 2명과 운전자가 탄 오토바이 2대 추가.
- 좌판 앞 중앙 통로의 기본 유효 폭 4.76m. 사람/오토바이 배치 구간은 이보다 좁다.
- 통로로 돌출된 좌판 2개: X=9.2–10.8m 구간의 유효 폭 1.80m.
- 쇼핑객이 모인 교차 구간: X=15.5–18.5m, 통과 경로가 중앙에서 옆으로 휘어진다.
- 하역 오토바이와 상자 실은 손수레가 마주 보는 구간: X=18.5–20.8m.
- 손수레 2개와 가게 앞 적재 상자 8개로 통로 주변 시야와 여유를 줄였다.
- 쇼핑 정차 표시 3곳, 출구 대기 공간. 과일 정차점은 좌판과 겹치지 않도록 X=8.2m로 조정.
- 입구 횡단보도: 보행 폭 2.8m, 도로 횡단 거리 4.2m, 흰색 줄무늬 7개.
- 양쪽 대기 공간·점자블록 표시·차량 정지선·횡단보도 표지판.
- 연석은 중앙 3.2m 진입부를 비워 휠체어 경로에 단차가 없다.
- 아스팔트·횡단보도 도색·점자블록은 시각적 표시이며 물리적 바닥은 평탄하다.
- 휠체어 시작 자세 `x=-6.2, y=0, z=0.03, yaw=0`, 진행 방향 +X.

**시장 내부는 고정 배치이며, 입구의 보행자 2명·오토바이 2대만 이동한다.**
이동 객체도 기존 단순 형상을 사용한다. 오토바이 운전자의 몸통·헬멧·팔·다리를 추가했다.
시나리오 위치·이동 주기는 `config/market_layout.json`에 있다.

## 입구 이동 시나리오

`ros2 launch swan_market market.launch.py`로 열면 기본으로 다음 동작이 반복된다.
시간 기준은 Gazebo 시뮬레이션 시간이므로 VM 성능에 따라 실제 시간은 더 길어진다.

- 0–15초: 오토바이 2대가 양쪽 차로를 약 0.99m/s로 이동한다. 도로 끝에서는
  연결된 곡선 경로로 돌아가며 위치를 순간적으로 되돌리지 않는다.
- 15–17초: 오토바이는 횡단보도에서 떨어진 위치에 대기한다.
- 17–27초: 보행자 2명이 서로 반대 방향으로 약 0.62m/s로 횡단한다.
- 27–32초: 대기 후 다음 주기를 시작한다. 보행자는 다음 주기에 반대편으로 돌아온다.

모델의 위치를 연속 갱신하는 **스크립트 방식**이다. 바퀴 동역학, 팔다리 보행 애니메이션,
휠체어를 감지한 양보·회피는 구현하지 않았다. 충돌 형상도 함께 이동하지만 접촉 물리의
정확성을 검증한 결과는 아니다. 교통 신호등 제어도 포함하지 않는다.
Gazebo를 일시정지하면 이동 시간도 정지한다.

이동을 끄고 초기 위치의 모델을 보려면 `moving_traffic:=false`를 사용한다.
기존 정적 장애물 주행 시험기는 이동 교통을 고려하지 않으므로 v4에서 실행을 거부한다.
동적 장애물 회피 성능을 시험하려면 별도의 판단·제어와 접촉 검증이 필요하다.

## 팀 공통 실행 방식

다른 월드와 같은 `wheelchair_gazebo simulation.launch.py`에서 `world`만 바꾼다.

```bash
ros2 launch wheelchair_gazebo simulation.launch.py world:=market
ros2 launch wheelchair_gazebo simulation.launch.py world:=layout_narrow_alley.world
ros2 launch wheelchair_gazebo simulation.launch.py world:=level1_basic.world
```

`market`은 `swan_market/worlds/market_shopping.sdf`의 별칭이다.
`world_package:=swan_market world:=market_shopping.sdf`로도 선택할 수 있다.
기존 `ros2 launch swan_market market.launch.py` 역시 공통 런처로 연결된다.
공통 옵션은 `headless`, `software_rendering`, `moving_traffic`, `spawn_x/y/z/yaw`다.
시장 선택 시에만 이동 교통 런처를 연결한다.
새 월드 추가 규칙과 센서 토픽 계약은 [공통 실행 안내](../wheelchair_gazebo/README.md)에 있다.

## ROS 없이 맵만 열기

Gazebo Harmonic이 설치된 Ubuntu에서 아래 경로를 이 패키지의 실제 경로로 바꾼다.

```bash
export MARKET_PKG=/absolute/path/to/swan_market
export GZ_SIM_RESOURCE_PATH="$MARKET_PKG/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r "$MARKET_PKG/worlds/market_shopping.sdf"
```

월드 자체에는 휠체어가 없다. 모든 맵 자산은 로컬에 포함되어 Fuel 다운로드가 필요 없다.
이 명령만으로는 이동 제어기가 실행되지 않아 교통 모델이 초기 위치에 머문다.
반복 이동은 아래 ROS 런치를 사용한다.

## 휠체어와 센서까지 열기

`swan_market`과 `wheelchair_gazebo`가 같은 ROS2 Jazzy 워크스페이스에 있어야 한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /absolute/path/to/swan
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash
ros2 launch swan_market market.launch.py
# UTM VM에서 소프트웨어 렌더링으로 실행
ros2 launch swan_market market.launch.py software_rendering:=true
# GUI 없이 실행
ros2 launch swan_market market.launch.py headless:=true
```

카메라 토픽은 모델 SDF에서 읽어 ROS `/camera`로 연결한다. 따라서 로컬 모델의
`/wheelchair/camera/image`와 팀 모델의 `/camera` 모두 지원하도록 작성했다.
출력은 `/scan`, `/camera`, `/odom`, `/tf`, `/clock`; 구동 입력은
`/model/wheelchair/cmd_vel`이다. 기존 simulation 런치와 동시에 실행하면 중복 로봇/토픽이
생기므로 이 런치를 시장 시뮬레이션 진입점으로 사용한다.

선택적으로 설치된 `teleop_twist_keyboard`로 수동 조종한다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/model/wheelchair/cmd_vel
```

## v3 횡단보도 주행 시험 기록

출발점 X=-6.2m에서 횡단보도 전 대기점 X=-5.6m를 거쳐 시장 쪽 X=0.8m까지
검증용 제어기로 바퀴 구동을 시험했다. 결과는 `preview/driving/crosswalk_run1/`에 있다.
새 런치는 `config/market_layout.json`의 spawn 값을 읽어 로봇을 배치한다.
진입 구간만 다시 시험하려면 아래 재실행 명령에 `--max-waypoints 2`를 추가한다.
이 기록은 이동 교통을 추가하기 전 v3에서 수행한 결과다.

## v2 휠체어 실제 주행 시험 기록 — 2026-09-14

Gazebo 실제 위치 피드백으로 ROS `/model/wheelchair/cmd_vel`을 제어하는
`tools/drive_market_test.py`를 실행했다. 주행 시작 전 입구에 배치한 이후에는
위치 이동 서비스를 사용하지 않았으며, 모든 이동은 바퀴 구동으로 수행했다.
이 제어기는 시험용이며 Nav2 또는 센서 기반 자율 위치추정 제어기가 아니다.

- 입구 X=-1m → 출구 X=33m의 예시 경로 11개 목표 도달(위치 허용오차 0.10m).
- 좌판 병목, 쇼핑객 밀집 구간, 하역/주차 오토바이 구간 통과.
- X=8, 12, 20, 33m 부근에서 정지 명령 후 선속도·각속도 0 확인.
- 90° 좌회전 목표에서 실제 88.6° 도달(오차 약 1.4°), 전방 방향으로 복귀.
- 약 0.416m 후진 후 최종 선속도·각속도 0 확인.
- 최대 odometry 선속도 0.28m/s. 회전 중 중심 이동과 후진을 포함한 누적 이동 34.84m.
- 727개 실제 위치 표본에서 반경 0.75m 검사 원의 최소 정적 장애물 여유 약 0.15m.
- 마지막 기록의 상대 odometry 이동량과 실제 위치 이동량 차이 약 4.9cm.

이는 표본 위치와 정적 SDF 충돌 형상으로 계산한 여유이며 접촉 센서 측정은 아니다.
측면 가게 앞 쇼핑 정차점 진입, 움직이는 사람/오토바이 회피는 시험하지 않았다.
시험 종료 후 시뮬레이션을 일시정지했다.

v2 근거: `preview/archive_v2/driving/run1/`의 결과·궤적·카메라 PNG. 기존 링크 유지를
위해 `preview/driving/run1/`에도 같은 기록을 보관한다. 각 실행 폴더의 `layout.json`,
`obstacles.json`은 당시 배치/충돌 형상이다. 현재 v3 검사 형상은
`preview/driving/obstacles.json`에 있다.
맵이나 로봇 형상이 바뀌면 검사 형상과 경로 여유를 먼저 다시 계산해야 한다.

재실행은 해당 시장 런치를 새로 시작한 뒤, **VM의 ROS 환경이 설정된 터미널**에서
패키지 소스 경로를 지정한다. 다른 주행 제어기를 동시에 실행하지 않는다.

```bash
export MARKET_SOURCE=/home/swan/market_verify_ws/src/swan_market
python3 "$MARKET_SOURCE/tools/drive_market_test.py" --execute \
  --layout "$MARKET_SOURCE/config/market_layout.json" \
  --geometry "$MARKET_SOURCE/preview/driving/obstacles.json" \
  --output /tmp/market-driving-test
```

## 카메라·LiDAR·YOLO 실시간 화면

`tools/sensor_monitor.py`는 `/camera` 원본 영상, `/scan`의 LiDAR 원점 기준 평면도,
팀 `best.pt`의 실제 YOLO 검출 영상을 한 창에 표시한다. `/yolo/image_raw`와
`/yolo/detected_objects`만 발행하며 휠체어 속도 명령은 보내지 않는다.
카메라·LiDAR·YOLO 각각의 수신 개수와 데이터 경과 시간을 표시한다.
CPU 추론은 416 입력 크기, 신뢰도 0.25 이상, 최대 2회/초로 제한한다.
검출 0개일 때도 빈 결과를 발행해 이전 검출이 남지 않도록 했다.

VM에서 ROS 환경을 설정한 뒤 실행한다(PyQt5, numpy, torch, ultralytics 필요).

```bash
python3 /home/swan/market_verify_ws/src/swan_market/tools/sensor_monitor.py \
  --model /home/swan/integration_ws/src/wheelchair_vision/wheelchair_vision/best.pt
```

실행 중 실제 횡단보도 `intact_crosswalk` 검출과 세 센서 화면 갱신을 확인했다.
전체 클래스의 인식 정확도를 평가한 결과는 아니다. LiDAR 뒤쪽 점에는 휠체어 자체가
포함될 수 있다. 조종은 별도의 `SWAN 휠체어 조종` 터미널에서 한다.

## 수정과 재생성

`tools/generate_market.py`가 SDF, 간판 COLLADA/PNG, 배치 JSON을 생성한다.
저장된 자산으로 실행할 때는 Pillow나 한글 폰트가 필요 없다.
재생성할 때만 Python Pillow와 한글 폰트가 필요하다.

```bash
python3 tools/generate_market.py --font /path/to/KoreanFont.ttf
```

생성기를 수정한 뒤 실행하면 생성된 맵/간판/JSON이 덮어써진다.

## 검증 범위

로컬에서 XML/Python 구문, 메시·텍스처 경로, 정적 장애물과 예시 중앙 경로의
기하학적 여유를 확인했다(이동 객체 제외). 로컬 휠체어의 충돌 형상 외접 반경은 약 0.73m이며,
반경 0.75m 원으로 경로를 검사하면 최소 여유는 0.15m다(양쪽 좌판 사이 1.80m, 검사 원 지름 1.50m). 정차 지점 3곳도
같은 원과 장애물이 겹치지 않는다. 근거는 `preview/validation.json`이다.
v4 장면은 공통 런처로 개편하기 전 UTM `SWAN-Jazzy`의 ROS2 Jazzy / Gazebo Harmonic에서 빌드·실행하고 SDF를 검사했다.
반복 이동의 실제 Gazebo 위치와 카메라·LiDAR 수신 결과는 `preview/traffic/`에 저장했다.
당시 실행 요약은 `preview/runtime_validation.json`에 있다.
공통 런처 개편 후에는 오프라인 월드 선택·생성·브리지 설정 테스트를 수행했다.
VM이 꺼져 있어 새 공통 런처의 Gazebo 실행은 아직 재검증하지 않았다.
`tools/check_crosswalk_traffic.py`로 2주기를 0.05초 간격으로 검사했으며,
이동 인물·오토바이 상호 간 및 기존 장애물과의 2D 형상 겹침이 없었다.
이 검사는 휠체어를 제외한다. 정적 예시 경로의 여유 검사 역시 이동 교통을 제외하므로,
현재 횡단보도를 무조건 통과할 수 있다는 의미가 아니다.
v3 입구 주행 기록은 `preview/driving/crosswalk_run1/`, v2 시장 내부 전체 주행 기록은
`preview/archive_v2/`에 남아 있다. v4 자율주행·동적 장애물 회피 검증 결과가 아니다.

실행 검증 중 SDF 링크 안의 collision/visual 이름 중복을 수정했다.
흰색 간판은 COLLADA에 발광·주변광 및 법선 정보를 명시하고 SDF PBR 텍스처를
지정한 뒤 정상 표시됨을 확인했다. UTM에서 Ogre2 렌더러가 종료되는 문제는
`software_rendering:=true` 옵션으로 해결했다. 이 옵션은 Mesa `llvmpipe`와
Qt `xcb`를 사용하고 `LIBGL_ALWAYS_SOFTWARE` 변수를 해제한다.
검증 VM에서는 소프트웨어 렌더링으로 실시간보다 느리게 실행된다.

검증에 사용한 VM 워크스페이스는 `/home/swan/market_verify_ws`다.
VM 데스크톱 터미널에서 다음과 같이 재실행할 수 있다(기존 시장 실행을 먼저 종료).

```bash
source /opt/ros/jazzy/setup.bash
source /home/swan/market_verify_ws/install/setup.bash
ros2 launch swan_market market.launch.py software_rendering:=true
```

`preview/market_plan.png`는 평면 배치도, `preview/market_isometric.png`는 SDF 기본
형상을 그린 공간 미리보기다. 둘 다 실제 Gazebo 화면 캡처는 아니다.
`tools/inspect_market.py`로 재검사/재생성할 수 있다(numpy, matplotlib, shapely 필요).

추가 검증 과제는 휠체어 접근 시 정지·양보 제어와 실제 접촉 응답이다.
현재 보행자와 오토바이는 정해진 시간표에 따라 위치를 갱신하며, 휠체어를 감지해 피하지 않는다.

# 시장 월드

실제 월드 파일: **`src/wheelchair_gazebo/worlds/market_shopping.world`**.
다른 팀 월드와 같은 폴더에 저장하며, 이 패키지는 시장 자산·이동 제어·생성 도구를 제공한다.

가게 8개, 고정 인물 21명, 주정차 오토바이 3대가 있는 전통시장이다.
입구 횡단보도에는 왕복 보행자 2명과 운전자가 탄 오토바이 2대를 추가했다.
전체 바닥은 46 × 24m, 가장 좁은 통로는 1.8m다. 횡단보도 진입부는 단차 없이 비웠다.

## 실행

워크스페이스를 빌드하고 `install/setup.bash`를 적용한 뒤 실행한다.
빌드 방법·다른 월드 선택·공통 토픽은 [공통 실행 안내](WORLDS.md)에 있다.

```bash
ros2 launch swan_market world.launch.py world:=market
# UTM 소프트웨어 렌더링
ros2 launch swan_market world.launch.py world:=market software_rendering:=true
# 이동 교통 끄기
ros2 launch swan_market world.launch.py world:=market moving_traffic:=false
```

기존 `ros2 launch swan_market market.launch.py` 명령도 유지한다.
기존 Gazebo 실행을 종료한 다음 다른 월드를 선택한다.
초기 휠체어 위치는 `(-6.2, 0, 0.03)`, 방향은 +X다.

## 이동 시나리오

시뮬레이션 시간 32초 주기로 반복한다. Gazebo를 일시정지하면 이동도 멈춘다.

| 시간 | 동작 |
|---|---|
| 0–15초 | 오토바이 2대가 양쪽 차로를 약 0.99m/s로 이동 |
| 15–17초 | 오토바이 대기 |
| 17–27초 | 보행자 2명이 반대 방향으로 약 0.62m/s로 횡단 |
| 27–32초 | 대기 후 다음 주기. 보행자는 반대편으로 복귀 |

모델 위치를 갱신하는 스크립트 방식이다. 바퀴 동역학·보행 관절 애니메이션,
신호등 제어·휠체어를 감지한 양보·회피는 포함하지 않는다.
접촉 물리와 자율주행 회피 성능을 검증한 결과는 아니다.

## 파일 역할

| 폴더 | 내용 |
|---|---|
| `../wheelchair_gazebo/worlds/` | 실제 시장 월드와 초기 위치·추가 런처 설정 |
| `models/` | 월드가 참조하는 간판 메시·텍스처 |
| `launch/` | 시장 단축 명령과 이동 교통 런처 |
| `config/` | 배치·경로·이동 주기 데이터 |
| `scripts/` | 실행용 이동 제어기·센서 모니터 |
| `tools/` | 개발용 생성기·형상 검사기. 빌드 설치 대상에서 제외 |

## 센서 화면과 수동 조종

ROS 환경에서 실행한다. 모니터는 PyQt5, numpy, torch, ultralytics와 별도의 팀 YOLO 모델이 필요하다.
카메라·LiDAR 화면과 YOLO 검출을 표시하며 속도 명령은 보내지 않는다.

```bash
python3 src/swan_market/scripts/sensor_monitor.py --model /path/to/best.pt
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/model/wheelchair/cmd_vel
```

## 수정·검사

워크스페이스 루트에서 실행한다. 생성기는 Pillow와 한글 폰트,
형상 검사기는 numpy, matplotlib, shapely가 필요하다.

```bash
python3 src/swan_market/tools/generate_market.py --font /path/to/KoreanFont.ttf
python3 src/swan_market/tools/inspect_market.py
python3 src/swan_market/tools/check_crosswalk_traffic.py
```

생성기는 월드·간판·설정을 덮어쓴다. 검사 결과·미리보기는 `preview/`에 생성하고 Git에서는 제외한다.
검사 범위·실제 장면 사진·예전 기록은 [검증 요약](../../docs/market/README.md)에 있다.
모든 맵 자산은 저장소에 포함되어 Fuel 다운로드가 필요 없다.

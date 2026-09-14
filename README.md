# SWAN 시뮬레이션

## 시작

ROS2 Jazzy / Gazebo Harmonic 환경의 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wheelchair_gazebo swan_market --symlink-install
source install/setup.bash
ros2 launch wheelchair_gazebo simulation.launch.py world:=market
```

`world:=layout_narrow_alley.world` 또는 `world:=level1_basic.world`처럼 이름을 바꿔 다른 월드를 선택한다.
UTM에서는 `software_rendering:=true`를 추가한다. 기존 실행을 종료한 뒤 새 월드를 실행한다.

## 필요한 곳만 보기

- [공통 런처·월드 추가 규칙](src/wheelchair_gazebo/README.md)
- [시장 구성·센서 화면·맵 수정](src/swan_market/README.md)
- [검증 요약·실제 장면 사진](docs/market/README.md)
- [기존 판단 파이프라인](src/swan_pipeline/README.md)

```text
src/
  wheelchair_gazebo/  공통 실행·휠체어 모델·기존 팀 월드
  swan_market/        시장 월드·간판 자산·이동 교통
  swan_interfaces/    공용 메시지
  swan_pipeline/      판단 파이프라인
  swan_webui/         웹 화면
docs/market/          검증 요약과 대표 사진 2장
```

원시 검증 기록은 Git 이력에 보관하며 새 `preview/` 출력은 커밋하지 않는다.

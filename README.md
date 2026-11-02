# SWAN — 승현 최종 작업 정리 (2026-11-02)

제출 브랜치는 **`seunghyun`**이다. 공지 작성자의 최신 `dayeong da3fd917`(2026-11-02 02:18 KST, `integration-final` 통합 포함)을 기준으로 팀 파일을 유지하고, 기존 통합 월드 작업 `58af269`·`475e54d`·`62452ad`를 이력 그대로 포함했다. 이미 `integration`에 있는 SDF를 공통 월드 경로에 다시 복사하지 않았다.

## 실행

Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 환경에서 새 폴더에 받는다. 기존 미커밋 작업 폴더는 그대로 두고, 아래 폴더가 이미 있다면 다른 새 이름을 쓴다.

```bash
git clone --branch seunghyun --single-branch https://github.com/pdy606/swan.git ~/swan_seunghyun_ws
cd ~/swan_seunghyun_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/swan_world_integration --ignore-src -r -y
colcon build --symlink-install --packages-select swan_world_integration
source install/setup.bash
ros2 launch swan_world_integration recording.launch.py start_zone:=apartment moving_traffic:=false
```

위 명령은 기존 연결 동선을 재현할 **별도 촬영 월드**를 연다. 자동으로 180.6m 경로를 주행하지는 않는다. 운전·종료·의존성 설치는 [Linux 실행 안내](docs/integration/LINUX_RUN_GUIDE.txt)를 참고한다.

최신 팀 전체 스택은 Nav2·Vision 등 ROS/Python 의존성이 준비된 작업공간에서 다음처럼 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to swan_bringup
source install/setup.bash
ros2 launch swan_bringup swan.launch.py
```

팀 런처는 `wheelchair_gazebo/worlds/wheelchair_world.sdf`와 최신 로봇·Vision·판단·주행을 사용한다. 별도 촬영 런처와 동시에 켜지 않는다. **11/2에는 VM·Gazebo·YOLO를 실행하지 않았으며, 이 최종 조합의 실행 성공·거리 정확도를 검증한 결과는 없다.**

## 제출 내용

- [통합 월드 패키지](src/swan_world_integration/README.md): 아파트·주차장·시장·도시와 기존 휠체어·센서를 SDF 한 개로 보존한다. 외부 mesh·모델 다운로드가 필요하지 않다. 재현용 로봇·제어기는 10월 버전이며 최신 팀 모델을 대체하지 않는다.
- [10/24 연결 동선 검증](docs/integration/CONNECTED_ROUTE_CHECK_20261024.txt): 정지 교통·고정 경로 명령으로 약 **180.635m**, 4/4 구간 이동. 중간 위치 강제 이동·드라이버 재시작 없음. [요약 수치](docs/integration/route_validation_20261024/summary.json), [검증 조건](docs/integration/route_validation_20261024/metadata.json), [주행 로그](docs/integration/route_validation_20261024/driver.log)를 보존했다.
- [API 비교자료](docs/integration/API_COMPARISON_20261019/README.txt): **10/19 조사 당시** 기능·가격·계산 가정. 11/2 가격 재확인, 유료 호출, 최종 모델 선정·주행 연동·응답 속도 실측은 수행하지 않았다. HTML은 내려받아 브라우저로 연다.
- [최종 전달 안내](docs/integration/SEUNGHYUN_HANDOFF_20261102.md): 변경 파일, 기존 영상·로그 위치, 이번 정적 검사와 남은 검증.

기존 영상은 `SWAN_seunghyun_handoff_20261102.zip`으로 별도 첨부한다. 영상 바이너리는 Git에 중복 추가하지 않는다. **180.6m 기록은 최신 Vision 거리 측정·Gap 정확도·신호 판단·장애물 회피·무충돌·자율주행의 검증 결과가 아니다.**

## 시뮬레이터 없이 검사

```bash
python3 -m unittest discover -s src/swan_world_integration/test -p 'test_*.py' -v
```

단일 SDF, 원본 모델·제어기 보존, 외부 자산 참조, 런처 연결과 임시 복사본 정리를 검사한다. 실제 주행·인식 정확도 검사를 대신하지 않는다. 기존 시장 단독 실험은 [과거 기록](docs/market/README.md)으로 보존하며 최신 실행 지침은 위 안내를 따른다.

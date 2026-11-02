# 승현 작업 전달 — 2026-11-02

제출 브랜치: `seunghyun`. 오늘 20시 전 개인 브랜치 정리·변경 파일 공유 공지를 따른다. 기준은 공지 작성자의 `dayeong da3fd91750955b6d4dd6f5e744b2baa0b186d381`(11/2 02:18 KST)이며 `integration-final`의 `9f1117b`를 포함한다.

## 반영 방식

- 기존 `seunghyun 011fa32` 이력에 통합 월드 작업 `58af269`·`475e54d`·`62452ad`를 한 번만 포함했다. 같은 커밋을 재작성하거나 이미 `integration`에 있는 SDF를 공통 월드 경로에 다시 복사하지 않았다.
- 팀 기준 파일은 `dayeong da3fd917`과 같은 내용·실행 권한으로 유지했다. 최신 Vision·모델·Nav2·판단·공통 런처를 과거 버전으로 되돌리지 않았다. `7e191a7`은 두 브랜치의 이력을 함께 보존하는 병합 커밋이다.
- 별도 `swan_world_integration`은 10월 고정 모델·제어기·통합 SDF를 보존하는 촬영 패키지다. 최신 팀 전체 스택의 월드나 로봇을 이 파일로 교체하지 않았다.
- 기존 로컬 미커밋 자료와 원래 작업 폴더는 그대로 보존하고 제출용 `seunghyun` 작업 폴더에서 문서를 갱신했다. `main` 직접 반영·강제 push·공유 브랜치 삭제는 하지 않는다.

## 승현 작업 파일

| 경로 | 내용 |
|---|---|
| `src/swan_world_integration/worlds/swan_town.sdf` | 아파트·주차장·시장·도시·연결로와 기존 휠체어·센서. 기존 검증 파일의 바이트 유지 |
| `src/swan_world_integration/config/` | 원본 커밋·해시·좌표와 시장 동작 배치 |
| `src/swan_world_integration/launch/`, `scripts/`, `tools/`, `test/`, `CMakeLists.txt`, `package.xml` | 보존한 촬영 실행·제어·검사·빌드 파일. 기존 커밋 내용 유지 |
| `README.md`, `src/swan_world_integration/README.md` | 최신 제출 브랜치, 팀 전체 실행과 촬영 실행의 차이, 검증 범위 |
| `docs/integration/LINUX_RUN_GUIDE.txt` | 새 폴더 설치·실행·키보드 운전·종료·선택적 YOLO 실행·최신 팀 실행 안내 |
| `docs/integration/CONNECTED_ROUTE_CHECK_20261024.txt` | 기존 180.635m 주행의 조건·한계·자료 위치 |
| `docs/integration/route_validation_20261024/` | 기존 `summary.json`, `metadata.json`, `status.json`, `driver.log`, `world.log` 원본 보존 |
| `docs/integration/API_COMPARISON_20261019/` | 당시 기능·비용 비교와 계산기, CSV·JSON·공식 출처·과거 로컬 시험. 11/2 신규 실측 자료가 아님을 표시 |
| `docs/integration/static_checks_20261102.json`, `static_tests_20261102.log` | 이번 정적 검사 결과와 20개 기존 검사 실행 로그 |

병합으로 새로 포함된 팀원 변경은 다영·규원·연호 작업이며 승현의 신규 개발로 보고하지 않는다. 예전 시장 실험 자료도 이력 보존을 위해 남겼으며 최신 팀 실행 지침은 루트 README를 따른다.

## 실행 명령

ROS Jazzy와 필요한 패키지가 설치된 Ubuntu 24.04 / Gazebo Harmonic에서 작업공간 루트 기준:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swan_world_integration
source install/setup.bash
ros2 launch swan_world_integration recording.launch.py start_zone:=apartment moving_traffic:=false
```

다른 터미널에서 ROS·작업공간을 source한 뒤:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/model/wheelchair/cmd_vel \
  -p speed:=0.15 -p turn:=0.3 -p repeat_rate:=10.0 -p key_timeout:=0.2
```

`k` 정지 → 운전 터미널 `Ctrl+C` → 월드 터미널 `Ctrl+C`. 자동으로 180.6m를 주행하는 명령은 아니다. 전체 설치·선택적 YOLO·최신 팀 전체 실행은 [Linux 안내](LINUX_RUN_GUIDE.txt)에 있다.

최신 팀 전체 스택의 진입점은 `ros2 launch swan_bringup swan.launch.py`다. 전체 의존성 준비 후 사용하며 촬영 런처와 동시에 켜지 않는다. 이번에 이 전체 조합을 실행해 확인하지 않았다.

## 기존 영상·로그 — 별도 첨부

공유 파일: **`SWAN_seunghyun_handoff_20261102.zip`**. 영상은 Git에 중복 업로드하지 않고 이 파일에 포함한다.

- `route_validation_20261024/report.html`: 경로·수치·영상·로그를 함께 보는 오프라인 보고서.
- `route_validation_20261024/route_camera_h264.mp4`: 10/24 카메라 표본 영상. 화면에 시뮬레이션 시각이 있다. 5fps·357초 영상이며 실제 실행 20.56분을 원속도로 녹화한 영상이 아니다.
- `route_validation_20261024/trajectory.csv`, `world.log`, `driver.log`, `summary.json`, `metadata.json`, `status.json`: 전체 궤적과 기존 실행 근거.
- `videos/SWAN_tour_highlights_72s.mp4`, `videos/SWAN_integrated_world_demo_180s.mp4`: 기존 10월 소개 영상. 최신 Vision의 새 실행 영상이 아니다.
- `docs/`: 이번 제출 안내·Linux 실행 안내·API 비교자료·검사 기록.

별도 보관된 `SWAN_route_validation_20261024.zip`에도 전체 주행 자료가 있다. 첨부 ZIP 안의 경로는 Git 저장소 내부 경로와 다르다. ZIP 전송은 사용자가 직접 수행한다.

## 검증 범위와 남은 항목

10/24는 정지 교통에서 고정 경로 명령으로 아파트 → 시장 끝 → 후진으로 도시 분기점 → 도시 북쪽 보행로를 약 **180.635m**, 4/4 구간 이동한 기록이다. 중간 위치 강제 이동·드라이버 재시작은 없다. 원본 월드와 제어기의 해시를 당시 설치 파일과 대조했다.

11/2에는 **VM·Gazebo·YOLO·유료 API를 실행하지 않고**, 원본 보존·자산 참조·런처의 기존 검사 20개, 파일 구문·문서 연결·원본 자료 대조를 수행했다. 실제 Linux 빌드·최신 전체 스택 실행을 검증했다고 보고하지 않는다.

남은 항목:

- 최신 Vision의 BEV·AIR 실제 거리 대비 MAE/RMSE, 객체 사이 Gap 정확도와 카메라 설정 보정.
- 최신 모델·전체 스택에서 신호 판단·인식 결과 소실·장애물 회피·동적 교통·정지·안전 기능 연동.
- 팀원 Linux PC의 새 설치·빌드·실행·종료 재현과 성능 확인.
- API 현재 가격·지원 조건 재확인, 실제 입력의 비용·응답 속도·인식 성능 비교, 최종 선정과 연동.
- 영상·자료의 실제 팀 전달 및 최종 통합 담당자의 검토·main 반영.

접촉 이벤트를 수집하지 않아 무충돌을 증명하지 않으며, 10/24 결과를 최신 Vision·자율주행 성공으로 확대하지 않는다. 10/18 YOLO 문제 기록도 당시 버전의 결과이며 최신 버전의 현재 결함으로 단정하지 않는다.

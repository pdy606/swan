# SWAN MBRA 반복 실험

ROS2 Jazzy + Gazebo Harmonic 환경에서 MBRA 단독(OFF)과 SWAN 보조(ON)를 반복 비교한다.
모델·주행·Nav2 연동·BLOCKED 복구·충돌 계측·평가 기준은 Phase7과 동일하다.

## 팀원 최초 설정

전체 SWAN 저장소를 클론한 뒤 `src/wheelchair_ai_driver` 폴더의 그래픽 세션 터미널에서:

```bash
./setup.sh
./experiment.sh start --off 20 --on 20
```

`setup.sh`는 공식 Hugging Face의 고정 revision에서 MBRA_PG_Official ONNX(253,629,409 bytes)를
다운로드하고 SHA-256을 검사한다. `.venv_mbra`에 Python 3.12 가상환경과
`requirements/cpu.lock.txt`의 CPU 의존성을 설치하고, 이 AI 패키지와 충돌 계측 플러그인만
`runtime/`에 빌드한다. ROS2 환경·공유 라이브러리·로컬 결과 저장 경로·6프레임 CPU 추론을 검증한 후에만
준비 완료를 표시한다. 가상환경 활성화나 별도 workspace 빌드는 필요 없다.

이미 유효한 모델·가상환경·빌드가 있으면 재사용한다. 기존 모델이나 관리 대상이 아닌 기존
가상환경이 불일치하면 덮어쓰지 않고 실패한다. 중단된 신규 다운로드는 다시 받을 수 있고,
이 설정 스크립트가 만들다 중단한 신규 가상환경만 다음 실행에서 설치를 이어간다.
재검증만 하려면 `./setup.sh --check`를 사용한다. 설정 기록은 `runtime/setup/report.json`이다.
실험 또는 다른 설정 작업이 실행 중이면 준비 작업을 거부한다.

**사전 설치:** Ubuntu 24.04, system Python 3.12/venv, ROS2 Jazzy, Gazebo Harmonic(Sim 8),
Nav2(특히 MPPI/Navfn), ros_gz, laser_filters, colcon, CMake, C++17 컴파일러,
Gazebo Sim/Plugin 개발 파일과 nlohmann_json. RAM 8GB 이상, 여유 디스크 1GB 이상,
최초 설치용 인터넷 연결과 카메라 렌더링용 X11 `DISPLAY`가 필요하다.
누락 시 필요한 apt 패키지를 안내하고 실패하며, sudo/apt 또는 시스템 Python 설치는 실행하지 않는다.

실험 기록은 **각 컴퓨터의 로컬 파일**이다. `results/` 전체를 Git에서 제외하므로 새 클론에는
다른 사용자의 CSV·JSON·로그·환경 지문이 없다. 폴더는 자동 생성하며 최초 `start`에서 해당
환경의 `experiment.json`을 만들고 OFF/ON 각각 1회차부터 시작한다. `setup.sh`는 저장 위치만
미리 확인한다. 모델·가상환경·빌드 역시 Git 제외 대상이며 `setup.sh`로 각 컴퓨터에서 준비한다.

기존 기록과 현재 지문(소스·모델·설정·충돌 플러그인 SHA-256)이 같으면 그대로 누적한다.
지문이 다르면 기존 기록을 보존하고 `results/environments/<지문>/`에서 새 실험을 시작한다.
이전에 사용했던 동일 지문으로 돌아오면 해당 폴더를 다시 사용한다. 지문 파일이나 기존 CSV를
다른 환경 값으로 덮어쓰지 않는다. 지문이 없거나 손상된 기존 결과도 합치거나 삭제하지 않는다.

## 실행

가상머신의 그래픽 세션에 연결된 터미널에서 실행한다. Gazebo GUI는 띄우지 않지만
기존 카메라 센서 렌더링에는 현재 환경의 DISPLAY 연결을 사용한다.
ROS2와 이 패키지의 환경은 스크립트가 자동으로 설정한다. MBRA 가상환경을 활성화하지 않는다.

```bash
cd showcase_main/src/wheelchair_ai_driver
./experiment.sh start
./experiment.sh status
./experiment.sh stop
```

`start`에서 OFF와 ON의 최종 누적 목표 횟수를 각각 입력한다. 예를 들어 두 조건 모두 3회가
저장돼 있을 때 각각 10을 입력하면 OFF 7회와 ON 7회를 추가한다. 서로 다른 목표도 가능하다.
인자를 지정해 대화 없이 실행할 수도 있다.

```bash
./experiment.sh start --off 10 --on 10
```

터미널을 닫아도 백그라운드에서 계속 실행한다. 이미 실행 중이면 중복 시작을 거부한다.
`status`의 완료 횟수는 도착 성공 수가 아니라 실패를 포함한 실험 결과 확정 수다.
목표를 모두 완료하면 결과를 저장하고 자신이 생성한 Gazebo·ROS2·추론 프로세스를 종료한다.
완료된 목표를 다시 입력하면 새 주행 없이 종료한다.

`stop`은 기존 계측 노드를 통해 emergency stop과 보조 OFF, 실제 정지를 요청하고 프로세스를
정리한다. 완료된 결과는 보존한다. 진행 중이던 미완료 회차는 평균에 넣지 않고, 다음
`start`에서 같은 번호를 새 시뮬레이터로 다시 시작한다. 종료되기 직전에 이미 확정된 정상
도착·실패·시간 초과 결과는 보존한다. 실험 중 오류는 `status`가 보여주는 결과 폴더의
`experiment.log`에서 확인한다.

## 결과

현재 결과 폴더는 `./experiment.sh status`로 확인한다. 최초 환경은 `results/`를 사용하며,
개발 PC의 기존 ON/OFF 각 20회 결과도 이 위치에 보존했다. 환경이 달라지면
`results/environments/<지문>/`을 사용하며,
각 폴더는 아래 파일을 독립적으로 관리한다.

- `swan_off.csv`: 실험 번호, 충돌 횟수, 목적지 도달률(%), 시뮬레이션 시간(s), 마지막 평균 행.
- `swan_on.csv`: 위 열과 자동 회피 개입 횟수, 마지막 평균 행.
- `records/`: 확정된 주행별 JSON. 재개와 중복 방지의 기준이며 삭제하지 않는다.
- `runs/`: 시도별 원본 접촉·odom·상태·노드 로그와 준비/종료 기록.
- `experiment.json`: 해당 환경의 실험 지문. 기존 값을 임의로 갱신하지 않는다.
- `experiment.log`: 백그라운드 실행 로그.
- `provenance/`: 결과 대조 검증 기록(검증 도구 실행 시 생성).

`runtime/active_results.json`은 현재 결과 폴더를 가리킨다. 기존 `results/PHASE7.md`와
`results/provenance/`의 과거 자료도 로컬에 보존한다. 새 클론에는 이 자료를 포함하지 않는다.
목표 횟수는 **현재 환경의 최종 누적 횟수**다. 같은 환경에 각 20회가 있으면 목표 20은
추가 실행 없이 종료하고, 결과가 없는 새 컴퓨터에서는 각각 20회를 실행한다.

매 주행 종료 직후 JSON과 CSV를 fsync 후 원자적으로 저장한다. CSV 저장 직전에 중단돼도
다시 실행하면 확정 JSON에서 복원한다. 실패와 시간 초과도 평균에 포함한다.

목적지 도달률은 통계적 성공률이 아닌 **코스 진행률(0~100%)**이다. 원래 스폰과 순서 있는
기존 경유지의 polyline에 대한 최대 진행 위치를 사용한다. 후진·우회 거리를 합산하지 않으며
모든 순서를 거쳐 실제 목적지 `(53,0)`에 도착한 경우에만 100%다.

충돌은 기존 12개 collision의 Gazebo 물리 접촉에서 외부 모델별 사건을 센다. 정상 주행면과
차체 내부 접촉을 제외하고, 실제 접촉 해제 후 다시 닿으면 새 사건이다. 평가 정보는 제어에
전달하지 않는다. ON 개입은 Nav2 목표 승인 후 첫 AVOIDING만 세며 같은 회피의 재계획은 중복하지 않는다.
시간은 준비 완료부터 도착/실패까지의 Gazebo 시뮬레이션 시간이다.

## 유지한 주행 조건

MBRA_PG_Official ONNX, RGB `/camera_driver` 320×240·10Hz·90°, 높이 1.2m·하향 7.5°,
최근 6프레임과 odom 상대 목표만 모델에 입력한다. 사용자 명령 상한은 0.30m/s다.
BLOCKED 복구 세션 수는 제한하지 않고, 각 세션은 최초 약 20cm + 추가 10cm씩,
누적 최대 50cm·15 시뮬레이션 초를 유지한다. 후방 확인 없는 복구는 Gazebo 전용이다.
Phase7 ON 1·3회차는 복구가 여러 번 성공한 뒤 개별 거리 한도로 종료됐으며, 해당 동작도 유지했다.

모델·경유지·속도·카메라·계측 코드 등 실험 조건이 달라지면 다음 시작 시 결과 폴더를 분리한다.
실험이 진행되는 도중 지문이 바뀌면 기존처럼 배치를 중단한다. 과거 폴더 이관 기록은 기존
`results/provenance/migration.json`에 보존했다.

## 구조 및 준비

```text
experiment.sh          start / status / stop
setup.sh               최초 다운로드·환경 준비·검증
wheelchair_ai_driver/  운전자·추론·상태·평가 공통 코드
launch/driving.launch.py
config/driving.yaml
evaluation/            원본 collision을 읽는 C++ 플러그인
assets/                공식 모델·입출력 정보·출처
.venv_mbra/            기존 MBRA 전용 Python 환경
requirements/          의존성 목록과 설치 버전
results/               로컬 전용 CSV·확정 결과·실행 기록 (Git 제외)
  environments/        환경이 바뀐 경우 지문별 독립 결과
runtime/               내부 빌드·설치·파생 월드·프로세스 상태
resource/              ROS2 패키지 등록
tools/                 실행 관리·결과 검증·접촉 검증
test/                  유지보수 회귀 테스트
```

`setup.sh`로 최초 준비한 뒤, `experiment.sh`는 설치된 모델·가상환경을 재사용한다.
빌드 산출물이 없으면 이 패키지만 runtime 아래에 자동 빌드한다. 원본 SWAN은 수정하지 않는다.
필요한 외부 환경은 ROS2 Jazzy, Gazebo Harmonic, 기존 Nav2/laser_filters/ros_gz 패키지와
Gazebo Sim 8·Plugin 2 개발 파일, CMake, C++17, nlohmann_json이다.

결과 대조 검사는 다음으로 실행한다.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD" python3 tools/verify_experiments.py
```

검증 도구도 현재 선택된 결과 폴더를 사용한다. 기존 PC의 물리 접촉 검증 요약은
`results/provenance/contact_validation.json`에 보존했으며 새 클론에는 포함하지 않는다.
`tools/validate_contacts.py`는 별도의 물리 검증용이며 반복 실험 시작에 필요하지 않다.

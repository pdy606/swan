# 로컬 LLM 연결 가이드

확인: **2026-11-03**, `seunghyun` 브랜치. 기존 ROS 코드 기준은 `b865185`다.
팀원이 이 저장소와 Ollama만으로 **오프라인 검사 → 선택적으로 Gazebo 연결**을 따라가는 안내다.
ROS 연결 단계는 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic과 기존 YOLO 실행 환경이 준비되어 있다고 가정한다. 새 Linux 환경 준비는 [Linux 실행 안내](LINUX_RUN_GUIDE.txt)를 참고한다.

## 1. 어디에 연결했나

| 파일 / 노드 | 역할 |
|---|---|
| [`ai_driver.py`](../../src/swan_market/scripts/ai_driver.py), ROS 이름 `ai_market_driver` | `/scan_filtered`, `/odom`, 지정한 목표를 읽어 Ollama에 질의하고 `/cmd_vel_user`에 제한된 조종 명령 발행 |
| [`ai_driver_core.py`](../../src/swan_market/scripts/ai_driver_core.py) | 입력 요약, JSON 응답 파싱, `stop/forward/left/right`와 거리 제한 처리 |
| 기존 `nav_avoidance_node` | 사용자/AI 명령, Nav2, 상황과 센서를 받아 최종 `/model/wheelchair/cmd_vel` 발행 |
| [`run_offline_check.py`](../../tools/local_llm/run_offline_check.py) | 이번 저장 이미지와 가정 센서 시험을 재현하는 독립 도구. ROS 노드가 아니며 주행 명령을 발행하지 않음 |

```text
/scan_filtered + /odom + CLI 목표
        → ai_market_driver ↔ Ollama(qwen3:8b)
        → /cmd_vel_user → 기존 주행 감독 → /model/wheelchair/cmd_vel

/camera → YOLO → 통합 상황 판단 → /driving_situation → 기존 주행 감독
```

**현재 AI 조종자 입력은 센서 텍스트다.** `/camera`, YOLO JSON, Gap을 LLM에 직접 보내지 않는다. 모델을 Gemma로 바꿔도 이 노드가 자동으로 이미지를 보내지는 않는다. 11/2의 66회 실험 역시 ROS 주행이 아닌 오프라인 검사였다.

## 2. 처음 한 번: 코드와 모델 준비

기존 작업 폴더를 덮지 않고 새 폴더에 받는다. 아래 경로가 이미 있으면 다른 새 이름을 사용한다.

```bash
git clone --branch seunghyun --single-branch https://github.com/pdy606/swan.git ~/swan_llm_ws
cd ~/swan_llm_ws
```

Linux는 [Ollama 공식 설치](https://docs.ollama.com/linux), Mac은 [공식 앱](https://ollama.com/download/mac)을 설치한다. Linux 설치 명령은 다음과 같다.

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen3:8b
ollama list
```

`ollama list`에서 서버 연결 오류가 나면 별도 터미널에서 `ollama serve`를 실행해 둔다. 이미 서버가 실행 중이면 다시 띄울 필요가 없다. Python용 `ollama` 패키지와 API 키는 필요 없다. 도구가 Python 기본 HTTP 기능으로 로컬 서버에 연결한다.

## 3. 먼저 오프라인 검사: ROS 없이 가능

```bash
cd ~/swan_llm_ws
python3 tools/local_llm/run_offline_check.py --model qwen3:8b --mode text
```

센서 8사례를 3회씩 검사한다. 공통 검사 프롬프트는 기존 ROS 조종자의 프롬프트와 다르다. `match=False`는 정한 기준과 모델 답변이 다르다는 뜻이며 설치 실패를 뜻하지 않는다. 결과 폴더는 터미널에 표시되고 기본 위치는 `/tmp/swan_llm_checks/`다. `results.jsonl`에 답변, `summary.json`에 일치 수와 시간을 저장한다.

이미지 모델도 비교하려면 **같은 컴퓨터의 Ollama**에 Gemma를 설치하고 실행한다.

```bash
ollama pull gemma3:4b
python3 tools/local_llm/run_offline_check.py --model gemma3:4b --mode text
python3 tools/local_llm/run_offline_check.py --model gemma3:4b --mode image
```

저장소에 포함한 원본 PNG 6장을 각 3회 검사한다. 별도 영상 파일, 개인 Mac 경로, VM이나 ROS가 필요 없다. 짧게 확인하려면 명령 끝에 `--repeats 1 --limit 1`을 붙인다.

**판독:** [11/2 실제 기록](local_llm_validation_20261102/)에서 Qwen 텍스트는 18/24, Gemma 텍스트는 12/24가 기대 행동과 일치했다. 두 모델 모두 가까운 장애물 앞에서 전진을 제안했고, Gemma는 신호 없는 장면을 초록으로 오판했다. 고정 사례의 반복이므로 일반 정확도나 주행 성공률이 아니다. 모델 태그의 내용이 바뀌면 결과도 달라질 수 있어 실행 시 digest를 기록한다.

## 4. 선택: 기존 ROS AI 조종자 연결

**Gazebo의 짧고 빈 평지 구간 시험용**이다. 기존 주행 스택이 정상 실행되는 환경에서 진행한다. AI와 키보드 조종자를 동시에 실행하지 않는다.

처음 빌드:

```bash
cd ~/swan_llm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to swan_bringup
```

**모든 ROS 터미널에서 아래 공통 준비를 실행한다.** `42`는 예시 도메인으로, 해당 시험의 터미널끼리 같게 맞추고 다른 로봇과 분리한다.

```bash
cd ~/swan_llm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

**터미널 A: 기존 전체 스택**

```bash
ros2 launch swan_bringup swan.launch.py
```

기본 launch에는 LLM이 포함되지 않는다. 별도 촬영용 launch나 두 번째 Gazebo는 함께 켜지 않는다.

**터미널 B: 센서, 목표 확인 후 AI 실행**

```bash
ros2 topic echo /odom --once --field pose.pose.position
ros2 service call /set_autonomy std_srvs/srv/SetBool "{data: true}"
```

목표는 **현재 `/odom` 좌표계**다. 시작 위치가 `(0, 0)` 부근이고 +x 방향에 빈 공간이 있을 때 다음 짧은 목표 예시를 사용한다. 시작 좌표나 통로 방향이 다르면 `--goal-x`, `--goal-y`를 그 공간에 맞춰 바꾼다.

```bash
/usr/bin/python3 src/swan_market/scripts/ai_driver.py \
  --endpoint http://127.0.0.1:11434/api/chat \
  --model qwen3:8b --scan-topic /scan_filtered \
  --goal-x 0.8 --goal-y 0.0 --max-wall-seconds 45 \
  --log /tmp/swan_ai_trial.csv
```

45초 또는 목표 허용 범위 도달 시 종료한다. 목표 예시는 전체 동선 완주 명령이 아니다. 신호, 장애물, 센서 상태에 따라 멈출 수 있다.

**터미널 C: 연결 확인**

```bash
ros2 node info /ai_market_driver
ros2 topic echo /cmd_vel_user --once
```

AI 터미널에는 `AI forward -> stop (front_clearance)`처럼 **모델 제안 → 제한 적용 결과**가 출력된다. 최종 명령은 주행 감독이 다시 결정한다. CSV의 `proposed`, `applied`, `gate`, `api_latency_s`로 원인을 확인한다.

`/set_autonomy`는 기본 ON이다. **false는 정지가 아니라 감독 기능을 우회하는 수동 모드**이므로 AI 시험의 정지 용도로 사용하지 않는다. 기존 내리막 모드도 자동 개입을 우회할 수 있어 평지 시험부터 한다. 이 가이드는 최신 전체 조합의 AI 주행 성능 검증 완료를 뜻하지 않는다.

## 5. UTM이면: Ollama는 Mac, ROS는 Ubuntu VM

Mac에서 2번 모델 준비를 하고, VM에서는 4번 ROS 준비를 한다. VM의 `127.0.0.1`은 Mac이 아니므로 기존 저장소의 [UTM 중계기](../../src/swan_market/tools/utm_ollama_proxy.py)를 사용한다.

VM에서 `hostname -I`, `ip route`로 VM 주소와 Mac 측 주소를 확인한다. 아래 `192.168.64.1`, `.5`는 과거 환경 예시이므로 실제 주소로 바꾼다.

**Mac 터미널:** 같은 저장소를 받은 경로에서 실행한다.

```bash
python3 ~/swan_llm_ws/src/swan_market/tools/utm_ollama_proxy.py \
  --bind 192.168.64.1 --allow-client 192.168.64.5
```

**VM의 AI 명령:** 4번 명령의 endpoint만 바꾼다.

```bash
/usr/bin/python3 src/swan_market/scripts/ai_driver.py \
  --endpoint http://192.168.64.1:11435/api/chat \
  --model qwen3:8b --scan-topic /scan_filtered \
  --goal-x 0.8 --goal-y 0.0 --max-wall-seconds 45 \
  --log /tmp/swan_ai_trial.csv
```

중계기는 허용한 VM의 `/api/chat` 텍스트 요청만을 위한 기존 도구다. 16KB 요청 제한이 있어 이미지 전달용으로 사용하지 않는다. 3번 이미지 검사는 Mac에서 직접 실행한다.

## 6. 종료와 자주 막히는 곳

1. **AI 터미널에서 Ctrl+C**. 노드는 정지 명령을 보내고 CSV를 닫는다.
2. 전체 스택 터미널에서 Ctrl+C. UTM 중계기도 본인이 띄웠다면 Ctrl+C.
3. 모델 메모리만 비우려면 다른 작업이 해당 모델을 사용하지 않을 때 `ollama stop qwen3:8b`. Ollama 서버 전체를 끌 필요는 없다.

| 증상 | 확인 |
|---|---|
| `Connection refused` | Ollama 실행 여부. UTM이면 endpoint가 Mac 중계기 주소인지 확인 |
| 모델을 찾지 못함 | `ollama list`, `ollama pull qwen3:8b` |
| AI 노드가 질의하지 않음 | `/scan_filtered`, `/odom` 수신과 모든 ROS 터미널의 도메인 확인 |
| 응답은 오는데 안 움직임 | 목표 좌표, CSV의 `gate`, 기존 `/driving_situation`, `/avoidance_status` 확인. 모드를 OFF로 바꿔 강제로 통과시키지 않음 |
| `rclpy`가 없음 | ROS 명령은 Ubuntu의 `/usr/bin/python3`로 실행하고 ROS setup을 source |

이번 추가는 문서와 독립 오프라인 검사 도구다. 기존 팀원의 소스, 모델, 공통 launch와 원본 월드는 변경하지 않았다. Ollama 설치와 모델 다운로드 외에는 저장소 자료만 사용하며, 유료 클라우드 API 설정은 필요 없다.

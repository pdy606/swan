# 시장 휠체어 실험: AI를 사람 조종자 대용으로 사용

2026-10-08에 UTM의 ROS 2 Jazzy / Gazebo Harmonic 시장 월드에서 검증했다. 실험자가 직접 조이스틱을 조작하기 어려울 때, **로컬 Qwen 3 8B가 짧은 조종 명령을 고르는 대용 조종자**가 된다. AI가 차량의 최종 속도 토픽을 직접 제어하지는 않는다. 목표와 장애물 정보를 받아 `stop`·`forward`·`left`·`right` 중 하나를 선택하고 `/cmd_vel_user`로 보낸다. 기존 `nav_avoidance_node.py`가 이를 받아 `/model/wheelchair/cmd_vel`을 발행한다.

```
Gazebo /odom + /scan → scan_filter → /scan_filtered
                                   ↘ AI 조종자 → /cmd_vel_user
                     /scan_filtered + /cmd_vel_user → 안전 감독 노드 → 휠체어
```

AI 호출은 [Ollama Chat API](https://docs.ollama.com/api/chat)의 로컬 HTTP 인터페이스를 사용한다. 출력은 [JSON schema](https://docs.ollama.com/capabilities/structured-outputs)로 네 행동에 제한했다. 소프트웨어의 별도 게이트는 전방 1.0m 미만, 회전 쪽 0.55m 미만, 목표 0.30m 이내에서 정지시키고, 센서가 2초 이상 끊기거나 API 응답이 없을 때도 정지시킨다. 명령은 최대 선속도 0.12m/s·각속도 0.35rad/s, 유효기간 1.5초이다. 이 수치는 **실험용 설정**이며 실제 휠체어 안전 인증을 뜻하지 않는다.

## 이번 검증 결과

UTM VM `SWAN-Jazzy`에서 시장 월드를 **한 번만** 실행하고, 이동 교통은 껐다(`moving_traffic:=false`). Mac의 Ollama `qwen3:8b`를 VM에서 호출했다. 원시 라이다 시험 후, 동일한 `swan` 사용자로 라이다 필터를 실행해 기본 `/scan_filtered` 경로를 다시 시험했다. 앞서 중복 실행했던 Gazebo의 데이터는 결과에서 제외했다.

| 조건 | 시작 `/odom` x | 목표 x | 최종 x | 실제 이동 | 목표 잔여 거리 | Qwen 응답 중간값 |
|---|---:|---:|---:|---:|---:|---:|
| 원시 `/scan` 시험 | 0.000m | 0.800m | 0.505m | 0.505m | 0.295m | 0.807초 (9회) |
| 필터 `/scan_filtered` + 안전 감독 노드 | 0.505m | 1.300m | 1.010m | 0.505m | 0.290m | 0.840초 (8회) |

두 시험 모두 `forward` 결정이 속도 명령과 오도메트리 이동으로 이어졌고, 0.30m 도착 기준에서 `stop`했다. [원시 라이다 CSV](pilot_2026-10-08.csv)와 [필터 경로 CSV](pilot_filtered_2026-10-08.csv)에 각 API 지연, 장애물 거리, 제안·적용 행동, 최종 위치를 보존했다. 카메라(320×240), 원시 라이다(720개 범위값), 필터 라이다, 오도메트리 수신도 확인했다.

이번 결과는 **정지한 교통 환경에서 짧은 직선 주행이 가능하다**는 증거다. 굽은 시장길, 좁은 병목, 움직이는 보행자·오토바이, 횡단보도 신호 판단, Nav2 장애물 우회 전환, YOLO 인식, 반복 성공률은 아직 검증하지 않았다. 현재 AI 입력은 LiDAR 방향별 최소거리와 오도메트리·목표뿐이므로 카메라 영상이나 신호등 색을 해석하지 못한다. 따라서 이 버전으로 신호 있는 횡단보도를 통과시키면 안 된다.

## 같은 조건으로 재현

모든 ROS 명령은 **VM의 같은 일반 사용자(`swan`)** 터미널에서 실행한다. `root`에서 띄운 필터와 `swan`에서 띄운 Gazebo를 섞으면 ROS 토픽·TF가 보이지 않을 수 있었다. 기존 Gazebo·안전 감독 노드를 종료해 각각 한 개만 남긴다. Mac에서 Ollama가 실행 중이고 `ollama list`에 `qwen3:8b`가 없다면 `ollama pull qwen3:8b`를 먼저 실행한다. 모델을 받은 뒤 추론에는 외부 유료 API 키가 필요 없다.

Mac 터미널에서 UTM 게스트 전용 중계기를 실행한다. 아래 IP는 이번 UTM 네트워크 값이므로 다른 VM에서는 게이트웨이·게스트 IP에 맞춰 `--bind`, `--allow-client`를 바꾼다.

```bash
python3 src/swan_market/tools/utm_ollama_proxy.py \
  --bind 192.168.64.1 --allow-client 192.168.64.5
```

VM의 저장소 루트에서 빌드한 후, 다음 명령을 각각 별도 터미널에 순서대로 실행한다. 각 터미널에서 `source /opt/ros/jazzy/setup.bash`와 `source install/setup.bash`를 적용한다.

```bash
colcon build --packages-select wheelchair_gazebo swan_market wheelchair_navigation --symlink-install
ros2 launch swan_market world.launch.py headless:=true software_rendering:=true moving_traffic:=false
ros2 launch wheelchair_navigation scan_filter.launch.py
ros2 launch wheelchair_navigation navigation.launch.py
ros2 launch wheelchair_navigation avoidance.launch.py
python3 src/swan_market/scripts/ai_driver.py \
  --endpoint http://192.168.64.1:11435/api/chat \
  --model qwen3:8b --scan-topic /scan_filtered \
  --goal-x 0.8 --goal-y 0 --max-wall-seconds 45 \
  --log /tmp/market_ai_trial.csv
```

`--goal-x`, `--goal-y`는 **Gazebo 세계 좌표가 아니라 현재 `/odom` 좌표**다. 새 월드를 열면 오도메트리 원점도 다시 잡힌다. 최종 행의 `gate=goal_reached`와 `x`,`y`,`goal_distance_m`을 확인한다. 명령을 중단할 때도 AI 노드는 영속도 명령을 보낸다. 완전한 센서·보행신호 시험 전에는 `moving_traffic:=false`를 유지한다.

## 10월 30일까지의 다음 실험 기준

이 README와 첫 주행 근거는 10월 8일에 작성했다. 10월 30일 발표·통합 실험에는 다음을 추가 측정해야 한다.

1. 같은 출발점·목표에서 최소 10회 반복해 도착률, 소요 시간, 경로 길이, 충돌·안전 정지 횟수와 API 지연의 중앙값·최댓값을 기록한다.
2. 시장 굽은 통로와 병목에서 정적 장애물 및 Nav2 우회 전환을 확인한다.
3. 이동 보행자·오토바이를 켠 실험은 별도 단계로 진행하고, 신호등 상태를 AI에 전달하거나 기존 신호 판단 노드와 연결한 뒤 적색 정지·초록 진행을 검증한다.
4. YOLO 카메라 인식과 판단 노드를 통합할 때는 검출 오류와 신호 오판 사례를 기록한다. 신호가 불확실하면 정지하도록 한다.

`gpt-6-astra` 유료 API는 이번에 호출하지 않았다. 로컬 Qwen이 목표 주행을 성공시켜 무료 API 대안의 작동 가능성을 먼저 확인했기 때문이다. 추가 모델 비교가 필요하면 같은 관측·목표·안전 게이트와 CSV 지표를 유지한 채 모델 어댑터만 교체한다.

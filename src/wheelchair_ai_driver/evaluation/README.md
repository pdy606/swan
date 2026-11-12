# 충돌 계측

`contacts.cc`는 기존 12개 collision의 ContactSensorData를 읽고 평가용 로컬 파일만 기록한다.
물리 형상·질량·관성·구동 및 MBRA/SWAN 제어 입력을 변경하지 않는다.
계측 구현과 충돌/진행률/시간 기준은 Phase7에서 유지했다.

일반 실행은 패키지 최상위의 `./experiment.sh start`를 사용한다.
빌드와 ROS 환경 준비는 자동 처리한다. 수동으로 플러그인만 빌드하려면:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S evaluation -B runtime/contact_build -DCMAKE_BUILD_TYPE=Release
cmake --build runtime/contact_build -j2
```

물리 검증 요약: `results/provenance/contact_validation.json`.
결과와 재개 방식은 패키지 최상위 README.md를 참고한다.

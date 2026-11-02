# SWAN Vision distance estimation 수정 요약

## 수정한 핵심 문제

1. `CvBridge(... desired_encoding='bgr8')` 뒤의 `RGB2BGR` 재변환 제거
   - 기존 코드는 R/B 채널이 다시 뒤집힐 수 있었음.

2. Ground BEV를 카메라 기하 기반 metric homography로 변경
   - 기본값은 현재 휠체어 모델의 camera 설정에 맞춤:
     - height: 0.64 m
     - pitch: 0.17 rad
     - horizontal FOV: 1.047 rad
   - bbox의 바닥 접점 픽셀을 `(X[m], Z[m])`로 직접 변환.

3. 공중 객체 분기 명확화
   - custom model의 `Red light STOP`, `green light GO`는 AIR 방식 사용.
   - 기존 `box_h + 0.5*dy` 개념은 유지.
   - X는 임의 배율 대신 실제 horizontal FOV로 계산.

4. Gap 계산 개선
   - 기존: Z 순서 이웃 객체의 중심-중심 대각선 거리.
   - 개선: ground 객체를 X 순서로 정렬하고, bbox 하단 좌우 edge를 BEV로 투영하여 실제 수평 free gap 계산.
   - Z 차이가 큰 객체는 서로 같은 통로상의 장애물로 보지 않도록 제외.

5. `/yolo/detected_objects` 형식 유지
   - integration-final과 호환되도록 `class`, `x`, `z` JSON을 그대로 발행.

## 실행

```bash
cd ~/swan
colcon build --packages-select wheelchair_vision --symlink-install
source install/setup.bash
ros2 launch swan_bringup swan.launch.py
```

## 확인 포인트

```bash
ros2 topic echo /yolo/detected_objects
```

화면 라벨은 다음처럼 보임:

```text
#1 Person Z:2.35m X:+0.31m [BEV]
#2 Red light STOP Z:4.10m X:-0.20m [AIR]
```

노란색 `Ground horizon` 선도 표시됨.

## 정확도 검증

Gazebo에서 객체를 실제 위치 2m, 3m, 4m, 5m 등에 배치하고 화면의 예측값을 기록한다.

`distance_validation_template.csv`의 `pred_x`, `pred_z`를 채운 뒤:

```bash
python3 tools/evaluate_distance.py distance_validation_template.csv
```

BEV와 AIR 각각의 MAE/RMSE를 확인할 수 있다.

## 조정 가능한 ROS parameter

- `confidence_threshold` = 0.20
- `max_dist_m` = 6.0
- `camera_height_m` = 0.64
- `camera_pitch_rad` = 0.17
- `camera_hfov_rad` = 1.047
- `bev_near_m` = 1.5
- `bev_half_width_m` = 0.8
- `air_scale` = 250.0
- `air_dy_weight` = 0.5
- `gap_depth_tolerance_m` = 1.5

특히 integration-final에서 `model.sdf`의 camera pose/FOV가 바뀌었다면 `camera_height_m`, `camera_pitch_rad`, `camera_hfov_rad`를 그 값에 맞춰야 한다.

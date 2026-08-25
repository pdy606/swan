# swan_pipeline — 통합 파이프라인 (노드 3개)

다영(YOLO) · 규원(Nav2) 브랜치와 합치는 최소 구성. 팀 계약만 이행.

## 노드
| 파일 | 역할 | 입력 → 출력 |
|------|------|-------------|
| `fusion_node.py` | **⑤ Camera+LiDAR 퓨전** (퍼셉션 코어) | `/scan_filtered` + `/yolo/detected_objects` → `/swan/detections` (물체=거리+클래스, 신호등=거리+빨강/초록) |
| `situation_node.py` | **C: 물리 장애물 회피** | `/swan/detections` → `/driving_situation="C"` (규원 Nav2), `/avoidance_status` 로 복귀 |
| `signal_node.py` | **B: 신호 의미기반** | `/swan/detections`(신호등) → `/driving_situation="B"`(빨강=정지) + `/signal_action` |

> A(위험없음=평상주행)는 발행하지 않음. `/driving_situation` 은 **B/C만**.

## 상황 판단 (다이어그램 ⑥)
```
        /scan_filtered ─┐
        /yolo ──────────┤→ fusion_node → /swan/detections
                              ├─→ situation_node → C (물리장애물 → Nav2 회피)
                              └─→ signal_node    → B (빨간불 → 감속/정지)
```

## 실행
```bash
ros2 launch swan_pipeline swan_integration.launch.py
# 전제: 팀이 sim + /scan_filtered + /yolo/detected_objects + Nav2 를 먼저 띄움
```

## 팀 계약 (인터페이스)
| 토픽 | 방향 | 내용 |
|------|------|------|
| `/scan_filtered` | 규원 → 나 | 자기몸체 제거된 LiDAR |
| `/yolo/detected_objects` | 다영 → 나 | YOLO 클래스 라벨 |
| `/swan/detections` | 나(내부) | 퓨전 결과 |
| `/driving_situation` | 나 → 규원 | "B"(신호정지) / "C"(회피) |
| `/avoidance_status` | 규원 → 나 | "COMPLETED" |
| `/signal_action` | 나 | 사람이 읽는 신호 행동 |

의존: `swan_interfaces` (Detection/DetectionArray/DriveTarget 메시지)

#!/usr/bin/env python3
"""
generate_worlds.py

전동휠체어 시뮬레이션용 다중 난이도 SDF 월드 생성기.

설계 근거: wheelchair_multi_world_design_rationale.md 참고.
Level 1(기본) ~ Level 4(미확인 검증 환경)까지, 장애물 밀도 / 통로 폭 /
연석·경사로 개수 / 정적·동적 보행자 수를 파라미터로 두어 하나의 스크립트에서
4개의 .world 파일을 자동 생성한다.

사용법:
    python3 generate_worlds.py
    (스크립트와 같은 디렉토리에 생성된 .world 파일이 저장됩니다)
"""

import random
from dataclasses import dataclass, field
from typing import List, Tuple

ROAD_LENGTH = 30.0
ROAD_HALF_WIDTH = 3.0
CROSSWALK_Y_RANGE = (-1.5, 1.5)


@dataclass
class LevelConfig:
    name: str
    description: str
    sidewalk_width: float
    obstacle_count: int
    obstacle_size_range: Tuple[float, float]
    narrow_zone: bool
    curb_ramp_count: int
    curb_height: float
    static_pedestrian_count: int
    dynamic_pedestrian_count: int
    seed: int


LEVELS: List[LevelConfig] = [
    LevelConfig(
        name="level1_basic",
        description="Level 1: 기본 인도 - 장애물 거의 없음, 평탄한 지형",
        sidewalk_width=3.0,
        obstacle_count=4,
        obstacle_size_range=(0.15, 0.20),
        narrow_zone=False,
        curb_ramp_count=0,
        curb_height=0.12,
        static_pedestrian_count=1,
        dynamic_pedestrian_count=0,
        seed=1,
    ),
    LevelConfig(
        name="level2_standard",
        description="Level 2: 표준 밀도 - 정적 장애물 표준 밀도, 연석 존재",
        sidewalk_width=3.0,
        obstacle_count=12,
        obstacle_size_range=(0.15, 0.22),
        narrow_zone=False,
        curb_ramp_count=1,
        curb_height=0.15,
        static_pedestrian_count=2,
        dynamic_pedestrian_count=0,
        seed=2,
    ),
    LevelConfig(
        name="level3_challenge",
        description="Level 3: 고밀도 challenge - 고밀도 장애물, 좁은 통로, 연석/경사로 다수",
        sidewalk_width=2.2,
        obstacle_count=24,
        obstacle_size_range=(0.15, 0.28),
        narrow_zone=True,
        curb_ramp_count=3,
        curb_height=0.18,
        static_pedestrian_count=3,
        dynamic_pedestrian_count=1,
        seed=3,
    ),
    LevelConfig(
        name="level4_unseen_eval",
        description="Level 4: 미확인 검증 환경 - 훈련 세트와 다른 배치, 정적+동적 보행자 혼합",
        sidewalk_width=2.6,
        obstacle_count=20,
        obstacle_size_range=(0.14, 0.30),
        narrow_zone=True,
        curb_ramp_count=2,
        curb_height=0.16,
        static_pedestrian_count=2,
        dynamic_pedestrian_count=2,
        seed=99,
    ),
]


def sdf_header() -> str:
    return '<?xml version="1.0" ?>\n<sdf version="1.9">\n'


def sdf_footer() -> str:
    return '</sdf>\n'


def world_open(name: str) -> str:
    return f'''  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

'''


def world_close() -> str:
    return '  </world>\n'


def box_model(name: str, pose: str, size: str, color: str, static: bool = True) -> str:
    return f'''    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


def cylinder_model(name: str, pose: str, radius: float, length: float, color: str, static: bool = True) -> str:
    return f'''    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


def sphere_model(name: str, pose: str, radius: float, color: str, static: bool = True) -> str:
    return f'''    <model name="{name}">
      <static>{str(static).lower()}</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
        <visual name="visual"><geometry><sphere><radius>{radius}</radius></sphere></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>
'''


def ground_and_road() -> str:
    parts = [
        box_model("ground_plane_visual", "0 0 -0.01 0 0 0", "100 100 0.02", "0.3 0.5 0.2"),
        box_model("road", "0 0 0.01 0 0 0", f"{ROAD_HALF_WIDTH * 2} {ROAD_LENGTH} 0.02", "0.15 0.15 0.15"),
    ]
    return "\n".join(parts)


def sidewalks(width: float) -> str:
    offset = ROAD_HALF_WIDTH + width / 2.0
    parts = [
        box_model("sidewalk_left", f"-{offset} 0 0.05 0 0 0", f"{width} {ROAD_LENGTH} 0.1", "0.7 0.7 0.7"),
        box_model("sidewalk_right", f"{offset} 0 0.05 0 0 0", f"{width} {ROAD_LENGTH} 0.1", "0.7 0.7 0.7"),
    ]
    return "\n".join(parts)


def crosswalk() -> str:
    stripes = []
    xs = [-2.4, -1.6, -0.8, 0.0, 0.8, 1.6, 2.4]
    for i, x in enumerate(xs, start=1):
        stripes.append(box_model(f"crosswalk_{i}", f"{x} 0 0.02 0 0 0", "0.5 3 0.01", "1 1 1"))
    return "\n".join(stripes)


def curbs_and_ramps(height: float, extra_ramp_count: int, rng: random.Random) -> str:
    parts = []
    south_len = ROAD_LENGTH / 2.0 - (CROSSWALK_Y_RANGE[1] - CROSSWALK_Y_RANGE[0]) / 2.0
    south_center = -(CROSSWALK_Y_RANGE[1] - CROSSWALK_Y_RANGE[0]) / 2.0 - south_len / 2.0
    north_center = (CROSSWALK_Y_RANGE[1] - CROSSWALK_Y_RANGE[0]) / 2.0 + south_len / 2.0

    for side, x in [("left", -ROAD_HALF_WIDTH - 0.1), ("right", ROAD_HALF_WIDTH + 0.1)]:
        parts.append(box_model(f"curb_{side}_south", f"{x} {south_center} {height/2 + 0.01} 0 0 0",
                                f"0.2 {south_len} {height}", "0.85 0.82 0.75"))
        parts.append(box_model(f"curb_{side}_north", f"{x} {north_center} {height/2 + 0.01} 0 0 0",
                                f"0.2 {south_len} {height}", "0.85 0.82 0.75"))

    parts.append(box_model("curb_ramp_left", f"-{ROAD_HALF_WIDTH + 0.1} 0 {height/2} 0 0.5 0",
                            f"0.4 3 {height}", "0.75 0.72 0.65"))
    parts.append(box_model("curb_ramp_right", f"{ROAD_HALF_WIDTH + 0.1} 0 {height/2} 0 -0.5 0",
                            f"0.4 3 {height}", "0.75 0.72 0.65"))

    for i in range(extra_ramp_count):
        y = rng.uniform(-ROAD_LENGTH / 2 + 3, ROAD_LENGTH / 2 - 3)
        side = rng.choice([-1, 1])
        x = side * (ROAD_HALF_WIDTH + 0.1)
        roll = 0.5 if side < 0 else -0.5
        parts.append(box_model(f"curb_extra_ramp_{i}", f"{x} {y:.2f} {height/2} 0 {roll} 0",
                                f"0.4 1.5 {height}", "0.75 0.72 0.65"))

    return "\n".join(parts)


def street_furniture(pedestrian_static: int, pedestrian_dynamic: int, rng: random.Random,
                      sidewalk_width: float) -> str:
    parts = []
    half = sidewalk_width / 2.0
    offset = ROAD_HALF_WIDTH + half

    tree_ys = [-10, -4, 3, 8]
    for i, y in enumerate(tree_ys, start=1):
        side = -1 if i % 2 == 1 else 1
        x = side * offset
        parts.append(box_model(f"tree_{i}", f"{x} {y} 0.15 0 0 0", "0.8 0.8 0.3", "0.4 0.25 0.15"))
        parts.append(cylinder_model(f"tree_{i}_trunk", f"{x} {y} 1.1 0 0 0", 0.15, 1.6, "0.35 0.2 0.1"))

    parts.append(box_model("bench_1", f"-{offset} -3 0.25 0 0 0", "1.5 0.5 0.5", "0.45 0.3 0.2"))
    parts.append(cylinder_model("trash_bin_1", f"-{offset} -13 0.3 0 0 0", 0.3, 0.6, "0.2 0.4 0.2"))
    parts.append(cylinder_model("hydrant_1", f"-{offset - 0.1} 5 0.25 0 0 0", 0.12, 0.5, "0.8 0.1 0.1"))

    for i, (x, y) in enumerate([(-offset + 0.9, -2), (offset - 0.9, 2), (-offset + 0.9, 2), (offset - 0.9, -2)], start=1):
        parts.append(cylinder_model(f"traffic_light_{i}", f"{x} {y} 1.5 0 0 0", 0.08, 3, "0.2 0.2 0.2"))

    for i in range(pedestrian_static):
        side = rng.choice([-1, 1])
        x = side * (offset - 0.4)
        y = rng.uniform(-ROAD_LENGTH / 2 + 2, ROAD_LENGTH / 2 - 2)
        parts.append(cylinder_model(f"pedestrian_static_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0", 0.22, 1.7,
                                     "0.35 0.35 0.4"))

    for i in range(pedestrian_dynamic):
        side = rng.choice([-1, 1])
        x = side * (offset - 0.4)
        y = rng.uniform(-ROAD_LENGTH / 2 + 2, ROAD_LENGTH / 2 - 2)
        parts.append(cylinder_model(f"pedestrian_dynamic_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0", 0.22, 1.7,
                                     "0.6 0.3 0.3", static=False))

    return "\n".join(parts)


def small_obstacles(count: int, size_range: Tuple[float, float], rng: random.Random,
                     sidewalk_width: float, narrow_zone: bool) -> str:
    parts = []
    half = sidewalk_width / 2.0
    offset = ROAD_HALF_WIDTH + half
    shapes = ["sphere", "box", "cylinder"]
    colors = {"sphere": "0.4 0.35 0.3", "box": "0.6 0.5 0.2", "cylinder": "0.5 0.5 0.5"}

    for i in range(count):
        side = rng.choice([-1, 1])
        base_x = side * offset
        x_jitter = rng.uniform(-(half - 0.3), (half - 0.3))
        x = base_x + x_jitter
        y = rng.uniform(-ROAD_LENGTH / 2 + 1.5, ROAD_LENGTH / 2 - 1.5)
        size = rng.uniform(*size_range)
        shape = rng.choice(shapes)
        z = size / 2 + 0.02

        if shape == "sphere":
            parts.append(sphere_model(f"obstacle_{i}", f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, colors["sphere"]))
        elif shape == "box":
            yaw = rng.uniform(0, 1.57)
            parts.append(box_model(f"obstacle_{i}", f"{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}",
                                    f"{size:.2f} {size:.2f} {size:.2f}", colors["box"]))
        else:
            parts.append(cylinder_model(f"obstacle_{i}", f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, size,
                                         colors["cylinder"]))

    if narrow_zone:
        narrow_y = 6.0
        gap = 0.85
        for side, sign in [("left", -1), ("right", 1)]:
            x = sign * offset + sign * (half - 0.4)
            parts.append(box_model(f"narrow_gate_{side}", f"{x:.2f} {narrow_y} 0.3 0 0 0",
                                    "0.4 0.4 0.6", "0.3 0.5 0.3"))
        for side, sign in [("left2", -1), ("right2", 1)]:
            x = sign * (ROAD_HALF_WIDTH + half) + sign * (half - 0.4)
            parts.append(box_model(f"narrow_gate_{side}", f"{x:.2f} {-narrow_y} 0.3 0 0 0",
                                    "0.4 0.4 0.6", "0.3 0.5 0.3"))

    return "\n".join(parts)


def buildings(sidewalk_width: float) -> str:
    offset = ROAD_HALF_WIDTH + sidewalk_width + 1.0
    parts = [
        box_model("building_left_1", f"-{offset} -8 1.5 0 0 0", "2 6 3", "0.6 0.4 0.3"),
        box_model("building_left_2", f"-{offset} 2 2 0 0 0", "2 8 4", "0.5 0.5 0.55"),
        box_model("building_right_1", f"{offset} -6 1.75 0 0 0", "2 7 3.5", "0.65 0.55 0.45"),
        box_model("building_right_2", f"{offset} 4 1.25 0 0 0", "2 5 2.5", "0.55 0.6 0.6"),
    ]
    return "\n".join(parts)


def build_world(cfg: LevelConfig) -> str:
    rng = random.Random(cfg.seed)

    sections = [
        f"    <!-- {cfg.description} -->\n",
        ground_and_road(),
        sidewalks(cfg.sidewalk_width),
        crosswalk(),
        curbs_and_ramps(cfg.curb_height, cfg.curb_ramp_count, rng),
        buildings(cfg.sidewalk_width),
        street_furniture(cfg.static_pedestrian_count, cfg.dynamic_pedestrian_count, rng, cfg.sidewalk_width),
        small_obstacles(cfg.obstacle_count, cfg.obstacle_size_range, rng, cfg.sidewalk_width, cfg.narrow_zone),
    ]

    body = "\n".join(sections)
    return sdf_header() + world_open(cfg.name) + body + "\n" + world_close() + sdf_footer()


def main():
    import os
    out_dir = os.path.dirname(os.path.abspath(__file__))
    for cfg in LEVELS:
        content = build_world(cfg)
        out_path = os.path.join(out_dir, f"{cfg.name}.world")
        with open(out_path, "w") as f:
            f.write(content)
        print(f"생성 완료: {out_path}  "
              f"(장애물 {cfg.obstacle_count}개, 인도폭 {cfg.sidewalk_width}m, "
              f"협착구간 {cfg.narrow_zone}, 정지보행자 {cfg.static_pedestrian_count}, "
              f"이동보행자 {cfg.dynamic_pedestrian_count})")


if __name__ == "__main__":
    main()

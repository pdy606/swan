#!/usr/bin/env python3
"""
generate_layouts.py

지형 "레이아웃" 다양성 축을 다루는 월드 생성기.
generate_worlds.py(밀도 축, Level 1~4)와 별개로, 통로의 기하학적 구조 자체가
다른 5종의 레이아웃을 생성한다.

레이아웃 5종:
  1. straight     : 직선형
  2. t_junction    : T자 교차형
  3. narrow_alley  : 좁은 골목형
  4. open_plaza    : 개방 광장형
  5. s_curve       : S자 커브형

사용법:
    python3 generate_layouts.py
"""

import random
import math
from dataclasses import dataclass
from typing import List, Tuple


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


def box_model(name, pose, size, color, static=True):
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


def cylinder_model(name, pose, radius, length, color, static=True):
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


def sphere_model(name, pose, radius, color, static=True):
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


def ground(size=60.0):
    return box_model("ground_plane_visual", "0 0 -0.01 0 0 0", f"{size} {size} 0.02", "0.3 0.5 0.2")


@dataclass
class DensityProfile:
    obstacle_count: int = 12
    obstacle_size_range: Tuple[float, float] = (0.15, 0.22)
    static_pedestrian_count: int = 2
    dynamic_pedestrian_count: int = 1


STANDARD_DENSITY = DensityProfile()


def scatter_obstacles(regions, count, size_range, rng, prefix):
    parts = []
    shapes = ["sphere", "box", "cylinder"]
    colors = {"sphere": "0.4 0.35 0.3", "box": "0.6 0.5 0.2", "cylinder": "0.5 0.5 0.5"}
    for i in range(count):
        x_min, x_max, y_min, y_max = rng.choice(regions)
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        size = rng.uniform(*size_range)
        shape = rng.choice(shapes)
        z = size / 2 + 0.02
        name = f"{prefix}_obstacle_{i}"
        if shape == "sphere":
            parts.append(sphere_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, colors["sphere"]))
        elif shape == "box":
            yaw = rng.uniform(0, 1.57)
            parts.append(box_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}",
                                    f"{size:.2f} {size:.2f} {size:.2f}", colors["box"]))
        else:
            parts.append(cylinder_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, size, colors["cylinder"]))
    return "\n".join(parts)


def scatter_pedestrians(regions, static_count, dynamic_count, rng, prefix):
    parts = []
    for i in range(static_count):
        x_min, x_max, y_min, y_max = rng.choice(regions)
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        parts.append(cylinder_model(f"{prefix}_pedestrian_static_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0",
                                     0.22, 1.7, "0.35 0.35 0.4"))
    for i in range(dynamic_count):
        x_min, x_max, y_min, y_max = rng.choice(regions)
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        parts.append(cylinder_model(f"{prefix}_pedestrian_dynamic_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0",
                                     0.22, 1.7, "0.6 0.3 0.3", static=False))
    return "\n".join(parts)


def build_straight(rng, density):
    road_len, road_hw, sw_w = 30.0, 3.0, 3.0
    parts = [
        box_model("road", "0 0 0.01 0 0 0", f"{road_hw*2} {road_len} 0.02", "0.15 0.15 0.15"),
        box_model("sidewalk_left", f"-{road_hw+sw_w/2} 0 0.05 0 0 0", f"{sw_w} {road_len} 0.1", "0.7 0.7 0.7"),
        box_model("sidewalk_right", f"{road_hw+sw_w/2} 0 0.05 0 0 0", f"{sw_w} {road_len} 0.1", "0.7 0.7 0.7"),
        box_model("crosswalk_1", "0 0 0.02 0 0 0", "3 3 0.01", "1 1 1"),
    ]
    left_region = (-road_hw - sw_w, -road_hw - 0.3, -road_len/2 + 1.5, road_len/2 - 1.5)
    right_region = (road_hw + 0.3, road_hw + sw_w, -road_len/2 + 1.5, road_len/2 - 1.5)
    regions = [left_region, right_region]
    parts.append(scatter_obstacles(regions, density.obstacle_count, density.obstacle_size_range, rng, "straight"))
    parts.append(scatter_pedestrians(regions, density.static_pedestrian_count, density.dynamic_pedestrian_count, rng, "straight"))
    return "\n".join(parts)


def build_t_junction(rng, density):
    main_len, branch_len, w = 24.0, 12.0, 2.6
    parts = [
        box_model("main_path", "0 0 0.01 0 0 0", f"{w} {main_len} 0.02", "0.55 0.55 0.55"),
        box_model("branch_path", f"{branch_len/2 + w/2} 0 0.01 0 0 0", f"{branch_len} {w} 0.02", "0.55 0.55 0.55"),
        box_model("junction_marker", "0 0 0.015 0 0 0", f"{w} {w} 0.005", "0.65 0.6 0.3"),
    ]
    main_region = (-w/2 + 0.2, w/2 - 0.2, -main_len/2 + 1.5, -w/2 - 0.3)
    branch_region = (w/2 + 0.3, branch_len + w/2 - 1.0, -w/2 + 0.2, w/2 - 0.2)
    regions = [main_region, branch_region]
    parts.append(scatter_obstacles(regions, density.obstacle_count, density.obstacle_size_range, rng, "tjunc"))
    parts.append(scatter_pedestrians(regions, density.static_pedestrian_count, density.dynamic_pedestrian_count, rng, "tjunc"))
    return "\n".join(parts)


def build_narrow_alley(rng, density):
    alley_len, alley_w = 20.0, 1.3
    parts = [
        box_model("alley_floor", "0 0 0.01 0 0 0", f"{alley_w} {alley_len} 0.02", "0.5 0.5 0.5"),
        box_model("wall_left", f"-{alley_w/2 + 0.5} 0 1.5 0 0 0", f"1 {alley_len} 3", "0.55 0.5 0.45"),
        box_model("wall_right", f"{alley_w/2 + 0.5} 0 1.5 0 0 0", f"1 {alley_len} 3", "0.55 0.5 0.45"),
    ]
    region = (-alley_w/2 + 0.15, alley_w/2 - 0.15, -alley_len/2 + 1.5, alley_len/2 - 1.5)
    regions = [region]
    reduced_count = max(1, density.obstacle_count // 2)
    parts.append(scatter_obstacles(regions, reduced_count, density.obstacle_size_range, rng, "alley"))
    parts.append(scatter_pedestrians(regions, density.static_pedestrian_count, 0, rng, "alley"))
    return "\n".join(parts)


def build_open_plaza(rng, density):
    plaza_size = 18.0
    parts = [
        box_model("plaza_floor", "0 0 0.01 0 0 0", f"{plaza_size} {plaza_size} 0.02", "0.6 0.6 0.6"),
    ]
    half = plaza_size / 2.0 - 1.0
    region = (-half, half, -half, half)
    regions = [region]
    increased_count = int(density.obstacle_count * 1.5)
    parts.append(scatter_obstacles(regions, increased_count, density.obstacle_size_range, rng, "plaza"))
    parts.append(scatter_pedestrians(regions, density.static_pedestrian_count, density.dynamic_pedestrian_count, rng, "plaza"))
    return "\n".join(parts)


def s_curve_point(t, amplitude, wavelength):
    x = amplitude * math.sin(2 * math.pi * t / wavelength)
    y = t
    return x, y


def s_curve_direction(t, amplitude, wavelength):
    dx = amplitude * (2 * math.pi / wavelength) * math.cos(2 * math.pi * t / wavelength)
    dy = 1.0
    norm = math.hypot(dx, dy)
    return dx / norm, dy / norm


def build_s_curve(rng, density, length=24.0, amplitude=2.5, wavelength=16.0, width=2.4, thickness=0.02, n_segments=40):
    parts = []
    half_len = length / 2.0
    ts = [(-half_len + i * (length / n_segments)) for i in range(n_segments + 1)]

    for i in range(n_segments):
        t0, t1 = ts[i], ts[i + 1]
        x0, y0 = s_curve_point(t0, amplitude, wavelength)
        x1, y1 = s_curve_point(t1, amplitude, wavelength)
        mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        yaw = -math.atan2(dx, dy)
        parts.append(box_model(
            f"scurve_tile_{i}",
            f"{mid_x:.3f} {mid_y:.3f} {thickness/2:.3f} 0 0 {yaw:.4f}",
            f"{width} {seg_len * 1.05:.3f} {thickness}",
            "0.55 0.55 0.55",
        ))

    lateral_margin = 0.25
    max_lateral = width / 2.0 - lateral_margin

    def scatter_along_curve(count, size_range, prefix):
        out = []
        shapes = ["sphere", "box", "cylinder"]
        colors = {"sphere": "0.4 0.35 0.3", "box": "0.6 0.5 0.2", "cylinder": "0.5 0.5 0.5"}
        for i in range(count):
            t = rng.uniform(-half_len + 1.0, half_len - 1.0)
            cx, cy = s_curve_point(t, amplitude, wavelength)
            dirx, diry = s_curve_direction(t, amplitude, wavelength)
            perp_x, perp_y = -diry, dirx
            lateral = rng.uniform(-max_lateral, max_lateral)
            x = cx + lateral * perp_x
            y = cy + lateral * perp_y
            size = rng.uniform(*size_range)
            shape = rng.choice(shapes)
            z = size / 2 + 0.02
            name = f"{prefix}_{i}"
            if shape == "sphere":
                out.append(sphere_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, colors["sphere"]))
            elif shape == "box":
                yaw = rng.uniform(0, 1.57)
                out.append(box_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}",
                                      f"{size:.2f} {size:.2f} {size:.2f}", colors["box"]))
            else:
                out.append(cylinder_model(name, f"{x:.2f} {y:.2f} {z:.2f} 0 0 0", size / 2, size, colors["cylinder"]))
        return "\n".join(out)

    def scatter_pedestrians_along_curve(static_count, dynamic_count, prefix):
        out = []
        for i in range(static_count):
            t = rng.uniform(-half_len + 1.5, half_len - 1.5)
            cx, cy = s_curve_point(t, amplitude, wavelength)
            dirx, diry = s_curve_direction(t, amplitude, wavelength)
            perp_x, perp_y = -diry, dirx
            lateral = rng.uniform(-max_lateral, max_lateral)
            x, y = cx + lateral * perp_x, cy + lateral * perp_y
            out.append(cylinder_model(f"{prefix}_ped_static_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0", 0.22, 1.7, "0.35 0.35 0.4"))
        for i in range(dynamic_count):
            t = rng.uniform(-half_len + 1.5, half_len - 1.5)
            cx, cy = s_curve_point(t, amplitude, wavelength)
            dirx, diry = s_curve_direction(t, amplitude, wavelength)
            perp_x, perp_y = -diry, dirx
            lateral = rng.uniform(-max_lateral, max_lateral)
            x, y = cx + lateral * perp_x, cy + lateral * perp_y
            out.append(cylinder_model(f"{prefix}_ped_dynamic_{i}", f"{x:.2f} {y:.2f} 0.85 0 0 0", 0.22, 1.7, "0.6 0.3 0.3", static=False))
        return "\n".join(out)

    parts.append(scatter_along_curve(density.obstacle_count, density.obstacle_size_range, "scurve_obstacle"))
    parts.append(scatter_pedestrians_along_curve(density.static_pedestrian_count, density.dynamic_pedestrian_count, "scurve"))
    return "\n".join(parts)


LAYOUT_BUILDERS = {
    "straight": build_straight,
    "t_junction": build_t_junction,
    "narrow_alley": build_narrow_alley,
    "open_plaza": build_open_plaza,
    "s_curve": build_s_curve,
}

LAYOUT_SEEDS = {
    "straight": 10,
    "t_junction": 11,
    "narrow_alley": 12,
    "open_plaza": 13,
    "s_curve": 14,
}

LAYOUT_DESCRIPTIONS = {
    "straight": "레이아웃 1: 직선형",
    "t_junction": "레이아웃 2: T자 교차형",
    "narrow_alley": "레이아웃 3: 좁은 골목형",
    "open_plaza": "레이아웃 4: 개방 광장형",
    "s_curve": "레이아웃 5: S자 커브형",
}


def build_layout_world(layout_name, density):
    rng = random.Random(LAYOUT_SEEDS[layout_name])
    builder = LAYOUT_BUILDERS[layout_name]
    body_parts = [
        f"    <!-- {LAYOUT_DESCRIPTIONS[layout_name]} -->\n",
        ground(),
        builder(rng, density),
    ]
    body = "\n".join(body_parts)
    return sdf_header() + world_open(layout_name) + body + "\n" + world_close() + sdf_footer()


def main():
    import os
    out_dir = os.path.dirname(os.path.abspath(__file__))
    for layout_name in LAYOUT_BUILDERS:
        content = build_layout_world(layout_name, STANDARD_DENSITY)
        out_path = os.path.join(out_dir, f"layout_{layout_name}.world")
        with open(out_path, "w") as f:
            f.write(content)
        print(f"생성 완료: {out_path}")


if __name__ == "__main__":
    main()

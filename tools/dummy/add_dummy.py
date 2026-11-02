#!/usr/bin/env python3
"""
SWAN 휠체어 탑승자 더미 (Hybrid III 50th 남성 충돌시험 더미 스타일)
사용법 (swan 저장소 루트에서):
  python3 tools/dummy/add_dummy.py           # 78 kg 장착 (기존 더미 교체)
  python3 tools/dummy/add_dummy.py 90        # 질량 지정
  python3 tools/dummy/add_dummy.py --remove  # 제거
visual은 상세 형상, collision은 박스 5개, 질량은 Hybrid III 분절 비율.
"""
import math, re, sys

SDF = "src/wheelchair_gazebo/models/wheelchair/model.sdf"
BEGIN, END = "<!-- BEGIN SWAN DUMMY -->", "<!-- END SWAN DUMMY -->"
SKIN, JACKET, JOINT, SHOE = "0.96 0.76 0.16", "0.10 0.13 0.20", "0.08 0.08 0.08", "0.15 0.15 0.15"

def mat(rgb):
    return f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><specular>0.25 0.25 0.25 1</specular></material>"
def f(*v):
    return " ".join(f"{x:.4f}" for x in v)
def rpy_z_to(d):
    dx, dy, dz = d; n = math.sqrt(dx*dx + dy*dy + dz*dz)
    return 0.0, math.acos(max(-1, min(1, dz / n))), math.atan2(dy, dx)

VIS = []
def vis(name, pose, geom, color):
    VIS.append(f'      <visual name="dummy_{name}"><pose>{pose}</pose><geometry>{geom}</geometry>{mat(color)}</visual>')
def capsule(name, a, b, r, color):
    d = [b[i] - a[i] for i in range(3)]; L = math.sqrt(sum(x*x for x in d))
    c = [(a[i] + b[i]) / 2 for i in range(3)]
    vis(name, f(*c, *rpy_z_to(d)), f"<capsule><radius>{r}</radius><length>{max(L-2*r, 0.001):.4f}</length></capsule>", color)
def sphere(name, c, r, color):
    vis(name, f(*c, 0, 0, 0), f"<sphere><radius>{r}</radius></sphere>", color)
def ellipsoid(name, c, radii, color, pitch=0.0):
    vis(name, f(*c, 0, pitch, 0), f"<ellipsoid><radii>{f(*radii)}</radii></ellipsoid>", color)
def box(name, c, size, color, rpy=(0, 0, 0)):
    vis(name, f(*c, *rpy), f"<box><size>{f(*size)}</size></box>", color)
def target(name, c, rpy, r=0.028):
    vis(name + "_ring", f(*c, *rpy), f"<cylinder><radius>{r+0.004}</radius><length>0.002</length></cylinder>", JOINT)
    vis(name + "_disk", f(*c, *rpy), f"<cylinder><radius>{r}</radius><length>0.003</length></cylinder>", SKIN)
    vis(name + "_barx", f(*c, 0, 0, 0), f"<box><size>{f(2*r, 0.005, 0.006)}</size></box>", JOINT)
    vis(name + "_barz", f(*c, 0, 0, 0), f"<box><size>{f(0.006, 0.005, 2*r)}</size></box>", JOINT)

for s, side in ((1, "l"), (-1, "r")):
    hip, knee, ankle = (-0.12, s*0.09, 0.64), (0.28, s*0.11, 0.62), (0.35, s*0.15, 0.27)
    shoulder, elbow, wrist = (-0.17, s*0.19, 1.10), (-0.11, s*0.24, 0.82), (0.13, s*0.25, 0.80)
    capsule(f"thigh_{side}", hip, knee, 0.075, SKIN)
    capsule(f"shin_{side}", knee, ankle, 0.055, SKIN)
    sphere(f"knee_{side}", knee, 0.062, JOINT)
    box(f"foot_{side}", (0.40, s*0.16, 0.235), (0.25, 0.10, 0.08), SHOE)
    sphere(f"shoulder_{side}", shoulder, 0.060, JOINT)
    capsule(f"upperarm_{side}", shoulder, elbow, 0.048, SKIN)
    sphere(f"elbow_{side}", elbow, 0.046, JOINT)
    capsule(f"forearm_{side}", elbow, wrist, 0.040, SKIN)
    ellipsoid(f"hand_{side}", (0.19, s*0.25, 0.79), (0.06, 0.035, 0.045), SKIN)
    target(f"tgt_head_{side}", (-0.135, s*0.098, 1.355), (math.pi/2, 0, 0))
    target(f"tgt_knee_{side}", (0.28, s*0.175, 0.62), (math.pi/2, 0, 0), r=0.022)
    target(f"tgt_shoulder_{side}", (-0.17, s*0.252, 1.10), (math.pi/2, 0, 0), r=0.024)
ellipsoid("pelvis", (-0.11, 0, 0.66), (0.14, 0.18, 0.10), SKIN)
capsule("abdomen", (-0.15, 0, 0.70), (-0.17, 0, 0.86), 0.12, SKIN)
ellipsoid("chest", (-0.17, 0, 0.98), (0.12, 0.19, 0.19), JACKET, pitch=-0.07)
box("sternum", (-0.055, 0, 0.98), (0.02, 0.12, 0.20), JOINT, rpy=(0, -0.07, 0))
capsule("neck", (-0.17, 0, 1.14), (-0.15, 0, 1.24), 0.045, JOINT)
ellipsoid("head", (-0.135, 0, 1.345), (0.100, 0.082, 0.118), SKIN)
ellipsoid("jaw", (-0.10, 0, 1.275), (0.065, 0.068, 0.05), SKIN)

COLL = [("thighs", f(0.07, 0, 0.635, 0, 0, 0), f(0.44, 0.36, 0.14)),
        ("torso", f(-0.16, 0, 0.93, 0, 0, 0), f(0.22, 0.38, 0.50)),
        ("head", f(-0.135, 0, 1.345, 0, 0, 0), f(0.19, 0.16, 0.23)),
        ("shins", f(0.32, 0, 0.44, 0, 0, 0), f(0.12, 0.38, 0.34)),
        ("feet", f(0.40, 0, 0.235, 0, 0, 0), f(0.25, 0.42, 0.08))]
SEG = [(0.059, (-0.135, 0, 1.345)), (0.019, (-0.16, 0, 1.19)), (0.225, (-0.17, 0, 0.98)),
       (0.290, (-0.12, 0, 0.70)), (0.054, (-0.14, 0, 0.96)), (0.060, (0.04, 0, 0.80)),
       (0.160, (0.08, 0, 0.63)), (0.133, (0.33, 0, 0.40))]

def block(M):
    tot = sum(r for r, _ in SEG)
    com = [sum(r * c[i] for r, c in SEG) / tot for i in range(3)]
    ixx = iyy = izz = 0.0
    for r, c in SEG:
        m = M * r / tot; dx, dy, dz = (c[i] - com[i] for i in range(3))
        ixx += m*(dy*dy + dz*dz); iyy += m*(dx*dx + dz*dz); izz += m*(dx*dx + dy*dy)
    ixx += 0.12*M/78; iyy += 0.12*M/78; izz += 0.10*M/78
    coll = "\n".join(f'      <collision name="dummy_{n}_collision"><pose>{p}</pose><geometry><box><size>{s}</size></box></geometry></collision>' for n, p, s in COLL)
    return f"""    {BEGIN}
    <!-- 탑승자 더미 (Hybrid III 50th 스타일), 질량 {M:.0f} kg, base_link 고정 -->
    <link name="dummy_occupant">
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
      <inertial>
        <pose>{f(*com, 0, 0, 0)}</pose>
        <mass>{M:.1f}</mass>
        <inertia><ixx>{ixx:.3f}</ixx><iyy>{iyy:.3f}</iyy><izz>{izz:.3f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
{coll}
{chr(10).join(VIS)}
    </link>
    <joint name="dummy_occupant_joint" type="fixed">
      <parent>base_link</parent>
      <child>dummy_occupant</child>
    </joint>
    {END}
"""

def strip(s):
    s = re.sub(r"[ \t]*" + re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n", "", s, flags=re.S)
    s = re.sub(r"\n[ \t]*<!--[^>]*탑승자 더미 \(yeonho\).*?<joint name=\"dummy_occupant_joint\".*?</joint>\n\n?", "", s, flags=re.S)
    return s

def main():
    src = strip(open(SDF, encoding="utf-8").read())
    if "--remove" in sys.argv:
        open(SDF, "w", encoding="utf-8").write(src); print("더미 제거 완료"); return
    M = float(next((a for a in sys.argv[1:] if not a.startswith("-")), 78))
    anchor = '    <link name="swan_slide_link">'
    assert src.count(anchor) == 1, "삽입 위치(swan_slide_link)를 못 찾음"
    open(SDF, "w", encoding="utf-8").write(src.replace(anchor, block(M) + anchor))
    print(f"더미 장착 완료: {M:.0f} kg, visual {len(VIS)}개, collision {len(COLL)}개")

if __name__ == "__main__":
    main()

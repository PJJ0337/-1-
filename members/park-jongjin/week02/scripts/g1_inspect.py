"""week02 월·화 실습: Unitree G1 MJCF 구조 분석
사용법:  python g1_inspect.py <mujoco_menagerie 경로> [출력폴더]
  - 바디 트리(텍스트)와 관절 표를 표준출력/파일로 저장
  - 오프스크린 렌더 2장(기본 자세, 어깨·팔꿈치를 움직인 자세) 저장
    (Windows에서는 뷰어로 직접 드래그: python -m mujoco.viewer --mjcf=<menagerie>/unitree_g1/scene.xml)
"""
import os, sys, math
import mujoco
import numpy as np

menagerie = sys.argv[1] if len(sys.argv) > 1 else "mujoco_menagerie"
out = sys.argv[2] if len(sys.argv) > 2 else "."
os.makedirs(out, exist_ok=True)
m = mujoco.MjModel.from_xml_path(os.path.join(menagerie, "unitree_g1", "scene.xml"))
d = mujoco.MjData(m)

JT = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}

def name(obj, i):
    return mujoco.mj_id2name(m, obj, i) or f"#{i}"

# ---------- 1) 바디 트리 ----------
children = {i: [] for i in range(m.nbody)}
for b in range(1, m.nbody):
    children[m.body_parentid[b]].append(b)

lines = []
def walk(b, prefix="", last=True):
    joints = [f"{name(mujoco.mjtObj.mjOBJ_JOINT, j)}({JT[m.jnt_type[j]]})"
              for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b])] if m.body_jntnum[b] else ["fixed"]
    label = f"{name(mujoco.mjtObj.mjOBJ_BODY, b)}  [{', '.join(joints)}]  m={m.body_mass[b]:.3f} kg"
    lines.append(prefix + ("└─ " if last else "├─ ") + label if b else "world")
    kids = children[b]
    for k, c in enumerate(kids):
        walk(c, prefix + ("   " if last else "│  ") if b else "", k == len(kids) - 1)
walk(0)
tree = "\n".join(lines)
open(os.path.join(out, "g1_body_tree.txt"), "w", encoding="utf-8").write(tree + "\n")
print(tree)

# ---------- 2) 요약 수치 ----------
robot_mass = sum(m.body_mass[b] for b in range(1, m.nbody) if name(mujoco.mjtObj.mjOBJ_BODY, b) != "floor")
print(f"\nnbody={m.nbody} njnt={m.njnt} nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom} 총질량={robot_mass:.3f} kg")

# ---------- 3) 어깨·팔꿈치(+손목) 관절 표 ----------
rows = []
for a in range(m.nu):
    j = m.actuator_trnid[a, 0]
    jn = name(mujoco.mjtObj.mjOBJ_JOINT, j)
    if not jn.startswith("left_") or not any(k in jn for k in ("shoulder", "elbow", "wrist")):
        continue
    lo, hi = m.jnt_range[j]
    frc = m.jnt_actfrcrange[j]
    rows.append((jn, m.jnt_axis[j], lo, hi, m.actuator_gear[a, 0], m.actuator_gainprm[a, 0],
                 m.actuator_ctrlrange[a], frc))
md = ["| 관절 | 축(바디 좌표) | 범위 [rad] | 범위 [deg] | gear | kp | ctrlrange | 토크 한계 [Nm] |",
      "|---|---|---|---|---|---|---|---|"]
for jn, ax, lo, hi, gear, kp, cr, frc in rows:
    md.append(f"| {jn} | ({ax[0]:.0f}, {ax[1]:.0f}, {ax[2]:.0f}) | {lo:.4f} ~ {hi:.4f} | {math.degrees(lo):.1f} ~ {math.degrees(hi):.1f} "
              f"| {gear:g} | {kp:g} | {cr[0]:.3f} ~ {cr[1]:.3f} | ±{frc[1]:g} |")
table = "\n".join(md)
open(os.path.join(out, "g1_arm_joints.md"), "w", encoding="utf-8").write(table + "\n")
print("\n" + table)

# ---------- 4) 렌더 (기본 / 관절 이동) ----------
try:
    r = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, 0.8]; cam.distance = 2.4; cam.azimuth = 150; cam.elevation = -15
    mujoco.mj_resetDataKeyframe(m, d, 0) if m.nkey else mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d); r.update_scene(d, cam); img0 = r.render()
    for jn, val in [("left_shoulder_pitch_joint", -1.5), ("left_shoulder_roll_joint", 0.8),
                    ("left_elbow_joint", 1.2), ("right_shoulder_roll_joint", -1.2)]:
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)]] = val
    mujoco.mj_forward(m, d); r.update_scene(d, cam); img1 = r.render()
    from PIL import Image
    Image.fromarray(np.hstack([img0, img1])).save(os.path.join(out, "g1_default_vs_moved.png"))
    print("\n렌더 저장:", os.path.join(out, "g1_default_vs_moved.png"))
except Exception as e:  # 디스플레이/GL이 없는 환경
    print("렌더 생략:", e)

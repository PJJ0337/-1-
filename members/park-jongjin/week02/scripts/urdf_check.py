"""week02 목요일 실습: 손으로 작성한 2링크 URDF를 MuJoCo로 열어 검증
사용법:  python urdf_check.py [URDF 경로] [출력폴더]
검증 항목
  1) 컴파일 성공 여부와 바디/관절 목록 (fixed joint 링크가 어떻게 처리되는지)
  2) 질량 · 관성이 URDF 값대로 들어왔는지
  3) 팔을 수평으로 폈을 때 중력 토크: 해석해 vs MuJoCo (qfrc_bias)
  4) 토크 0으로 놓았을 때 1초 뒤 자세 (자유 낙하 거동)
"""
import os, sys, math
import numpy as np
import mujoco

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "models", "two_link_arm.urdf")
out = sys.argv[2] if len(sys.argv) > 2 else "."
g = 9.81

def report(m, title):
    print(f"\n=== {title} ===")
    print(f"nbody={m.nbody} njnt={m.njnt} nq={m.nq} nu={m.nu}")
    for b in range(m.nbody):
        print(f"  body {b}: {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b):10s} "
              f"mass={m.body_mass[b]:.4f}  diaginertia={np.round(m.body_inertia[b], 6)}")
    for j in range(m.njnt):
        print(f"  joint {j}: {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j):10s} "
              f"axis={m.jnt_axis[j]} range={np.round(m.jnt_range[j], 4)} damping={m.dof_damping[m.jnt_dofadr[j]]:g}")

# ---- 1) 기본 로드 (MuJoCo는 URDF에 대해 fusestatic=true가 기본) ----
m = mujoco.MjModel.from_xml_path(path)
report(m, "기본 로드 (fusestatic 기본값)")
has_tool = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tool0") >= 0
print("tool0 바디 존재:", has_tool)

# ---- 1-b) tool0 보존: <mujoco><compiler fusestatic="false"/></mujoco> 삽입 ----
xml = open(path, encoding="utf-8").read()
xml2 = xml.replace('<robot name="two_link_arm">',
                   '<robot name="two_link_arm">\n  <mujoco><compiler fusestatic="false" discardvisual="false"/></mujoco>')
m2 = mujoco.MjModel.from_xml_string(xml2)
report(m2, "fusestatic=false")
print("tool0 바디 존재:", mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "tool0") >= 0)

# ---- 3) 수평 자세 중력 토크 ----
d = mujoco.MjData(m2)
d.qpos[:] = 0  # 두 링크 모두 +x 방향 수평
mujoco.mj_forward(m2, d)
m1, L1, m2_, L2 = 1.0, 0.30, 0.6, 0.25
tau_sh = g * (m1 * L1 / 2 + m2_ * (L1 + L2 / 2))
tau_el = g * (m2_ * L2 / 2)
print("\n=== 수평 자세 중력 토크 [Nm] ===")
print(f"  해석해  shoulder={tau_sh:.4f}  elbow={tau_el:.4f}")
print(f"  MuJoCo  shoulder={d.qfrc_bias[0]:.4f}  elbow={d.qfrc_bias[1]:.4f}  (qfrc_bias, 부호는 축 방향 기준)")
tool = mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_BODY, "tool0")
print(f"  tool0 위치 = {np.round(d.xpos[tool], 4)} (기대: [0.55, 0, 0.10])")

# ---- 4) 토크 0 자유 낙하 1초 ----
d.qpos[:] = 0; d.qvel[:] = 0
traj = []
while d.time < 1.0:
    mujoco.mj_step(m2, d)
    traj.append((d.time, *d.qpos))
print(f"\n=== 토크 0, 1초 후 === shoulder={math.degrees(d.qpos[0]):.1f} deg, elbow={math.degrees(d.qpos[1]):.1f} deg "
      f"(timestep={m2.opt.timestep}s)")
np.savetxt(os.path.join(out, "two_link_drop.csv"), np.array(traj), delimiter=",",
           header="time,shoulder_rad,elbow_rad", comments="", fmt="%.5f")

# ---- 렌더 ----
try:
    from PIL import Image
    r = mujoco.Renderer(m2, 360, 480)
    cam = mujoco.MjvCamera(); cam.lookat[:] = [0.25, 0, 0.1]; cam.distance = 1.3; cam.azimuth = 90; cam.elevation = -5
    imgs = []
    for q in ([0, 0], [-0.6, 1.0]):
        dd = mujoco.MjData(m2); dd.qpos[:] = q; mujoco.mj_forward(m2, dd); r.update_scene(dd, cam); imgs.append(r.render())
    Image.fromarray(np.hstack(imgs)).save(os.path.join(out, "two_link_arm_mujoco.png"))
except Exception as e:
    print("렌더 생략:", e)

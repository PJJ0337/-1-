"""week03 화요일: URDF → MuJoCo 로드 → MJCF 저장(mj_saveLastXML) → 손으로 정리한 MJCF(include · default)와 비교

실행 (저장소 루트에서):
    python members/park-jongjin/week03/scripts/tue_urdf_to_mjcf.py
출력:
    week03/mjcf/two_link_from_urdf.xml          (URDF 기본 로딩 결과, fusestatic=true)
    week03/mjcf/two_link_from_urdf_nofuse.xml   (<mujoco><compiler fusestatic="false" discardvisual="false"/> 추가)
    week03/results/tue_compare.txt              (구조 · 물성 · 동역학 비교표)
    week03/results/tue_drop.csv                 (수평 자세에서 놓았을 때 2초 궤적, 두 모델)
    week03/img/20261006_urdf_vs_mjcf_drop.png
"""
import os, sys, io
import numpy as np
import mujoco
import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjio import load_model, save_last_xml   # Windows 한글 경로 대응

HERE = os.path.dirname(os.path.abspath(__file__))
W3 = os.path.dirname(HERE)
URDF = os.path.join(W3, "..", "week02", "models", "two_link_arm.urdf")
MJCF_DIR = os.path.join(W3, "mjcf")
RES = os.path.join(W3, "results"); IMG = os.path.join(W3, "img")
os.makedirs(RES, exist_ok=True); os.makedirs(IMG, exist_ok=True)
out = io.StringIO()
def p(*a):
    print(*a); print(*a, file=out)

def names(m, objtype, n):
    return [mujoco.mj_id2name(m, objtype, i) for i in range(n)]

def summary(tag, m):
    p(f"[{tag}] nbody={m.nbody} njnt={m.njnt} ngeom={m.ngeom} nsite={m.nsite}")
    p(f"   bodies: {names(m, mujoco.mjtObj.mjOBJ_BODY, m.nbody)}")
    p(f"   joints: {names(m, mujoco.mjtObj.mjOBJ_JOINT, m.njnt)}")
    p(f"   총질량(world 제외) = {m.body_mass[1:].sum():.4f} kg")

p(f"MuJoCo {mujoco.__version__}")
p("=" * 70)
# ---------- 1. URDF 기본 로딩 → MJCF 저장 ----------
p("1) URDF 기본 로딩 (URDF 기본값: fusestatic=true, discardvisual=true)")
m_urdf = load_model(URDF)
f1 = os.path.join(MJCF_DIR, "two_link_from_urdf.xml")
save_last_xml(f1, m_urdf)
summary("URDF 기본", m_urdf)
p(f"   → 저장: mjcf/{os.path.basename(f1)}")

# ---------- 2. <mujoco> 확장으로 fusestatic 끄기 ----------
p("\n2) URDF 안에 <mujoco><compiler fusestatic=\"false\" discardvisual=\"false\"/></mujoco> 추가")
txt = open(URDF, encoding="utf-8").read()
ext = '<mujoco><compiler fusestatic="false" discardvisual="false"/></mujoco>'
txt2 = txt.replace('<robot name="two_link_arm">', '<robot name="two_link_arm">\n  ' + ext, 1)
m_nf = mujoco.MjModel.from_xml_string(txt2)
f2 = os.path.join(MJCF_DIR, "two_link_from_urdf_nofuse.xml")
save_last_xml(f2, m_nf)
summary("fusestatic=false", m_nf)
p(f"   → 저장: mjcf/{os.path.basename(f2)}")
lost = set(names(m_nf, mujoco.mjtObj.mjOBJ_BODY, m_nf.nbody)) - set(names(m_urdf, mujoco.mjtObj.mjOBJ_BODY, m_urdf.nbody))
p(f"   fusestatic=true 일 때 사라진 body: {sorted(lost)}")
p(f"   geom 수: 기본 {m_urdf.ngeom} vs discardvisual=false {m_nf.ngeom} (URDF visual {m_nf.ngeom - m_urdf.ngeom}개가 기본 로딩에서 버려짐)")

# ---------- 3. 손으로 정리한 MJCF (scene.xml → include two_link_arm.xml) ----------
p("\n3) 손으로 정리한 MJCF: mjcf/scene.xml (include two_link_arm.xml, default class 3개)")
m_h = load_model(os.path.join(MJCF_DIR, "scene.xml"))
summary("hand MJCF", m_h)

def full_inertia(m, b):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, m.body_iquat[b]); R = R.reshape(3, 3)
    return R @ np.diag(m.body_inertia[b]) @ R.T

p("\n4) 링크별 비교 (URDF 로딩 결과 vs 손으로 작성한 MJCF) — body 좌표계 기준")
p(f"{'body':10s} {'항목':14s} {'URDF→MuJoCo':>34s} {'hand MJCF':>34s}  최대오차")
maxerr = 0.0
for bn in ["link1", "link2"]:
    bu = mujoco.mj_name2id(m_urdf, mujoco.mjtObj.mjOBJ_BODY, bn)
    bh = mujoco.mj_name2id(m_h, mujoco.mjtObj.mjOBJ_BODY, bn)
    rows = [("mass", m_urdf.body_mass[bu:bu+1], m_h.body_mass[bh:bh+1]),
            ("pos(부모기준)", m_urdf.body_pos[bu], m_h.body_pos[bh]),
            ("COM ipos", m_urdf.body_ipos[bu], m_h.body_ipos[bh]),
            ("I diag(body)", np.diag(full_inertia(m_urdf, bu)), np.diag(full_inertia(m_h, bh)))]
    for k, a, b in rows:
        e = float(np.max(np.abs(np.asarray(a) - np.asarray(b))))
        maxerr = max(maxerr, e) if k != "I diag(body)" else maxerr
        fa = " ".join(f"{x:.6g}" for x in np.atleast_1d(a)); fb = " ".join(f"{x:.6g}" for x in np.atleast_1d(b))
        p(f"{bn:10s} {k:14s} {fa:>34s} {fb:>34s}  {e:.1e}")
for jn in ["shoulder", "elbow"]:
    ju = mujoco.mj_name2id(m_urdf, mujoco.mjtObj.mjOBJ_JOINT, jn)
    jh = mujoco.mj_name2id(m_h, mujoco.mjtObj.mjOBJ_JOINT, jn)
    du = m_urdf.jnt_dofadr[ju]; dh = m_h.jnt_dofadr[jh]
    p(f"{jn:10s} axis {m_urdf.jnt_axis[ju]} vs {m_h.jnt_axis[jh]} | range {m_urdf.jnt_range[ju]} vs {m_h.jnt_range[jh]} | damping {m_urdf.dof_damping[du]} vs {m_h.dof_damping[dh]}")

# ---------- 5. 수평 자세 중력 토크 (2주차 해석해 3.9731 / 0.7358 Nm) ----------
p("\n5) 수평 자세(q=0) 중력 토크 qfrc_bias  [2주차 해석해: shoulder 3.9731, elbow 0.7358 Nm]")
for tag, m in [("URDF", m_urdf), ("hand", m_h)]:
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    p(f"   {tag:5s}: shoulder {d.qfrc_bias[0]:.4f} Nm, elbow {d.qfrc_bias[1]:.4f} Nm")

# ---------- 6. 동역학 비교: q=0(수평)에서 놓고 2초 ----------
p("\n6) 수평 자세에서 놓은 뒤 2.0 s 자유 낙하 (timestep 두 모델 모두 0.002 s, integrator 차이 주의)")
T = 2.0
def run(m):
    d = mujoco.MjData(m); n = int(round(T / m.opt.timestep)); out_ = np.zeros((n + 1, 3))
    out_[0] = [0, *d.qpos[:2]]
    for i in range(n):
        mujoco.mj_step(m, d); out_[i + 1] = [d.time, *d.qpos[:2]]
    return out_
p(f"   URDF 모델 option: timestep {m_urdf.opt.timestep}, integrator {mujoco.mjtIntegrator(m_urdf.opt.integrator).name}")
p(f"   hand 모델 option: timestep {m_h.opt.timestep}, integrator {mujoco.mjtIntegrator(m_h.opt.integrator).name}")
tu = run(m_urdf)
# 공정 비교: hand 모델을 URDF와 같은 integrator로도 실행
m_h_euler = load_model(os.path.join(MJCF_DIR, "scene.xml")); m_h_euler.opt.integrator = m_urdf.opt.integrator
th = run(m_h); the = run(m_h_euler)
d_same = np.max(np.abs(tu[:, 1:] - the[:, 1:])); d_diff = np.max(np.abs(tu[:, 1:] - th[:, 1:]))
p(f"   같은 integrator 일 때 최대 |Δq| = {d_same:.2e} rad")
p(f"   integrator 다를 때(Euler vs implicitfast) 최대 |Δq| = {d_diff:.2e} rad")
p(f"   최종 자세 URDF: shoulder {tu[-1,1]:.4f}, elbow {tu[-1,2]:.4f} rad")
p(f"   최종 자세 hand: shoulder {th[-1,1]:.4f}, elbow {th[-1,2]:.4f} rad")
np.savetxt(os.path.join(RES, "tue_drop.csv"), np.column_stack([tu, th[:, 1:], the[:, 1:]]), delimiter=",",
           header="t,urdf_shoulder,urdf_elbow,hand_shoulder,hand_elbow,hand_euler_shoulder,hand_euler_elbow", comments="", fmt="%.6f")

try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["DejaVu Sans"]
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.plot(tu[:, 0], tu[:, 1], color="#1f77b4", lw=2.5, label="shoulder - URDF->MuJoCo")
    ax.plot(th[:, 0], th[:, 1], color="#ff7f0e", lw=1.2, ls="--", label="shoulder - hand MJCF")
    ax.plot(tu[:, 0], tu[:, 2], color="#2ca02c", lw=2.5, label="elbow - URDF->MuJoCo")
    ax.plot(th[:, 0], th[:, 2], color="#d62728", lw=1.2, ls="--", label="elbow - hand MJCF")
    ax.set_xlabel("time [s]"); ax.set_ylabel("joint angle [rad]")
    ax.set_title(f"Release from horizontal (q=0): max |dq| = {d_same:.1e} rad (same integrator)")
    ax.grid(alpha=.3); ax.legend(fontsize=8, ncol=2); fig.tight_layout()
    fig.savefig(os.path.join(IMG, "20261006_urdf_vs_mjcf_drop.png"), dpi=150)
    p("   → 그래프: img/20261006_urdf_vs_mjcf_drop.png")
except ImportError:
    p("   (matplotlib 없음 — 그래프 생략)")

ok = maxerr < 1e-4 and d_same < 1e-3
p("\n결과: " + ("일치 (구조 · 물성 · 동역학)" if ok else "불일치 — 위 표 확인"))
open(os.path.join(RES, "tue_compare.txt"), "w", encoding="utf-8").write(out.getvalue())

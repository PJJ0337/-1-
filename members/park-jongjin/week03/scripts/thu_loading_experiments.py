"""week03 목요일: '우리 로봇 URDF를 MuJoCo로 불러올 때' 체크리스트 근거 실험

실행 (저장소 루트에서):
    python members/park-jongjin/week03/scripts/thu_loading_experiments.py
실험:
    E1 URDF joint type → MuJoCo joint type 변환 (revolute · continuous · prismatic · planar · floating)
    E2 자기 충돌: 2주차 URDF에 base collision 을 되살렸을 때 부모(월드 고정)-자식 접촉 + exclude 로 해결
    E3 fusestatic on/off: body 수 · 이름, 동역학(중력 토크) 동일 여부
    E4 MJCF angle 기본값(degree) 함정: compiler angle 을 빼면 range 가 어떻게 바뀌는가
    E5 베이스 표현: 고정 / freejoint / 홀로노믹(slide x · slide y · hinge z) — nq · nv, 명령 좌표계
    E6 mjSpec attach: 홀로노믹 베이스(상판 치수) + 2링크 팔 2개 조립 → mjcf/holonomic_base_two_arms.xml 저장
출력: results/thu_experiments.txt, results/thu_holonomic.csv, mjcf/holonomic_base_two_arms.xml
"""
import os, io, re
import numpy as np
import mujoco
import sys; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjio import load_model, load_spec, read_text   # Windows 한글 경로 대응

HERE = os.path.dirname(os.path.abspath(__file__)); W3 = os.path.dirname(HERE)
URDF = os.path.join(W3, "..", "week02", "models", "two_link_arm.urdf")
MJ = os.path.join(W3, "mjcf"); RES = os.path.join(W3, "results")
out = io.StringIO()
def p(*a): print(*a); print(*a, file=out)
JT = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
def jtypes(m): return [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j), JT[int(m.jnt_type[j])]) for j in range(m.njnt)]
def bodies(m): return [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(m.nbody)]
urdf_txt = open(URDF, encoding="utf-8").read()
def with_ext(txt, ext): return txt.replace('<robot name="two_link_arm">', '<robot name="two_link_arm">\n  ' + ext, 1)

p(f"MuJoCo {mujoco.__version__}")
# ---------------- E1 ----------------
p("\nE1) URDF joint type → MuJoCo")
link = lambda n: f'<link name="{n}"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>'
for jt, extra in [("revolute", '<limit lower="-1" upper="1" effort="10" velocity="2"/>'), ("continuous", ""),
                  ("prismatic", '<limit lower="-0.1" upper="0.1" effort="10" velocity="1"/>'), ("planar", ""), ("floating", "")]:
    u = f'<robot name="t">{link("a")}{link("b")}<joint name="j" type="{jt}"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>{extra}</joint></robot>'
    try:
        m = mujoco.MjModel.from_xml_string(u)
        lim = [bool(m.jnt_limited[j]) for j in range(m.njnt)]
        p(f"   {jt:10s} → {jtypes(m)}  nq={m.nq} nv={m.nv} limited={lim}")
    except Exception as e:
        p(f"   {jt:10s} → 오류: {str(e).splitlines()[0]}")

# ---------------- E2 ----------------
p("\nE2) 자기 충돌: base_link collision 복원(원통 r0.05 h0.10, 어깨 z=0.10 바로 아래) 후 q=0에서 1 s 놓기")
coll = '<collision><origin xyz="0 0 0.05"/><geometry><cylinder radius="0.05" length="0.10"/></geometry></collision>\n  </link>\n\n  <link name="link1">'
u_col = urdf_txt.replace('</link>\n\n  <link name="link1">', coll, 1)
assert u_col != urdf_txt, "base collision 삽입 실패"
def drop(m, T=1.0):
    d = mujoco.MjData(m); mujoco.mj_forward(m, d); n0 = d.ncon
    for _ in range(int(T / m.opt.timestep)): mujoco.mj_step(m, d)
    return n0, d.qpos[0]
cases = []
m_a = mujoco.MjModel.from_xml_string(u_col); cases.append(("URDF 기본(fusestatic=true): base→world 로 합쳐짐", m_a))
m_b = mujoco.MjModel.from_xml_string(with_ext(u_col, '<mujoco><compiler fusestatic="false"/></mujoco>')); cases.append(("fusestatic=false: base_link 는 world 에 용접된 정적 body", m_b))
sp = mujoco.MjSpec.from_string(with_ext(u_col, '<mujoco><compiler fusestatic="false"/></mujoco>'))
sp.add_exclude(bodyname1="base_link", bodyname2="link1"); m_c = sp.compile(); cases.append(('+ <contact><exclude body1="base_link" body2="link1"/>', m_c))
for name, m in cases:
    n0, q = drop(m); p(f"   {name:58s}: 초기 접촉 {n0}개, 1 s 후 shoulder {q:+.3f} rad")
p("   → 원인: MuJoCo 문서(Computation > Collision > Selection, 필터 3) '부모-자식 body 접촉은 제외, 단 부모가 world 이면 제외 안 함.")
p("     joint 없이 용접된 body 들은 한 body 로 취급' → base_link 는 world 와 용접 = world 취급이라 link1 과의 접촉이 남는다.")

# ---------------- E3 ----------------
p("\nE3) fusestatic on/off (2주차 URDF 원본)")
m_on = load_model(URDF)
m_off = mujoco.MjModel.from_xml_string(with_ext(urdf_txt, '<mujoco><compiler fusestatic="false"/></mujoco>'))
for tag, m in [("on (URDF 기본)", m_on), ("off", m_off)]:
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    p(f"   {tag:14s}: nbody={m.nbody} {bodies(m)}  중력토크 q=0: {d.qfrc_bias[0]:+.4f}, {d.qfrc_bias[1]:+.4f} Nm")
p("   → 동역학 동일, tool0 · base_link 이름이 사라짐 → 이후 MJCF 에서 site · sensor · exclude · attach 대상으로 쓸 수 없음")

# ---------------- E4 ----------------
p("\nE4) MJCF compiler angle 기본값 = degree (URDF 는 radian)")
arm = open(os.path.join(MJ, "two_link_arm.xml"), encoding="utf-8").read()
m_rad = mujoco.MjModel.from_xml_string(arm)
m_deg = mujoco.MjModel.from_xml_string(arm.replace('angle="radian" ', ''))
p(f"   angle=radian 명시 : shoulder range = {m_rad.jnt_range[0]} rad")
p(f"   angle 생략(degree): shoulder range = {m_deg.jnt_range[0]} rad  (= ±{np.degrees(m_deg.jnt_range[0][1]):.4f}°)")

# ---------------- E5 · E6 ----------------
p("\nE5/E6) 베이스 표현 3종 + mjSpec attach 로 팔 2개 조립")
PLATE = (0.575, 0.472, 0.012)   # 노션 4주차: BASE 3.STEP 상판 575 × 472 × 12 mm (팀장 확인 10/2)
def build(base_mode):
    s = mujoco.MjSpec(); s.compiler.degree = False
    s.option.timestep = 0.002; s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    s.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05])
    s.worldbody.add_light(pos=[0, -1, 2.5], dir=[0, 0.4, -1])
    base = s.worldbody.add_body(name="base", pos=[0, 0, 0.30])
    if base_mode == "free":
        base.add_freejoint(name="base_free")
    elif base_mode == "holonomic":
        for n, t, ax in [("base_x", mujoco.mjtJoint.mjJNT_SLIDE, [1, 0, 0]), ("base_y", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0]),
                         ("base_yaw", mujoco.mjtJoint.mjJNT_HINGE, [0, 0, 1])]:
            base.add_joint(name=n, type=t, axis=ax, damping=1.0)
    # 하체: 상판(실치수) + 임시 박스(높이 0.25 m, 질량 20 kg 가정 — 하체 실측표로 교체 예정)
    base.add_geom(name="plate", type=mujoco.mjtGeom.mjGEOM_BOX, size=[PLATE[0]/2, PLATE[1]/2, PLATE[2]/2], mass=3.7, rgba=[.75, .75, .8, 1])
    base.add_geom(name="chassis", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.25, 0.20, 0.13], pos=[0, 0, -0.14], mass=20, rgba=[.3, .3, .35, 1])
    for side, y in [("left_", 0.18), ("right_", -0.18)]:
        arm_spec = load_spec(os.path.join(MJ, "two_link_arm.xml"))   # 한 번 붙이면 원본 spec 에서 빠지므로 팔마다 새로 읽음
        fr = base.add_frame(pos=[0.15, y, PLATE[2]/2])
        fr.attach_body(arm_spec.body("base_link"), side, "")
    if base_mode == "holonomic":
        for n in ["base_x", "base_y", "base_yaw"]:
            a = s.add_actuator(name=n + "_vel", target=n, trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_velocity(kv=200.0); a.forcerange = [-200, 200]; a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    for side in ["left_", "right_"]:
        for j, f in [("shoulder", 40), ("elbow", 20)]:
            a = s.add_actuator(name=f"{side}{j}_pos", target=f"{side}{j}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
            a.set_to_position(kp=50.0, kv=1.0); a.forcerange = [-f, f]; a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return s
for mode in ["fixed", "free", "holonomic"]:
    s = build(mode); m = s.compile()
    p(f"   {mode:10s}: nq={m.nq:2d} nv={m.nv:2d} nu={m.nu:2d}  joints={[n for n, _ in jtypes(m)][:4]}{' ...' if m.njnt > 4 else ''}")
    if mode == "holonomic":
        xml = s.to_xml(); open(os.path.join(MJ, "holonomic_base_two_arms.xml"), "w", encoding="utf-8").write(xml)
        p("   → 저장: mjcf/holonomic_base_two_arms.xml (mjSpec.to_xml)")
        p(f"   팔 prefix 확인: {[n for n in bodies(m) if 'link' in n]}")
        # 명령 좌표계 시험: 0~1 s 제자리 90° 회전(wz), 1~2 s vx=0.3 명령
        d = mujoco.MjData(m); ax = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ["base_x_vel", "base_y_vel", "base_yaw_vel"]}
        rows = []
        for i in range(int(2.0 / m.opt.timestep)):
            t = d.time; d.ctrl[:] = 0
            if t < 1.0: d.ctrl[ax["base_yaw_vel"]] = np.pi / 2
            else: d.ctrl[ax["base_x_vel"]] = 0.3
            mujoco.mj_step(m, d); rows.append([d.time, *d.qpos[:3]])
        rows = np.array(rows); np.savetxt(os.path.join(RES, "thu_holonomic.csv"), rows, delimiter=",", header="t,x,y,yaw", comments="", fmt="%.5f")
        yaw = rows[-1, 3]; x, y = rows[-1, 1], rows[-1, 2]
        p(f"   명령: 0–1 s yaw 속도 π/2 rad/s, 1–2 s 'base_x' 속도 0.3 m/s")
        p(f"   결과: yaw = {np.degrees(yaw):.1f}°, 위치 x = {x:.3f} m, y = {y:.3f} m")
        p(f"   → slide 축은 world 고정: 로봇이 90° 돌아 있어도 base_x 명령은 world x(로봇 기준 오른쪽 옆)로 이동")
        p(f"     body 기준 명령은 [vx_w, vy_w] = R(yaw)·[vx_b, vy_b] 로 바꿔 넣어야 함 (예: yaw 90°에서 전진 0.3 → base_y +0.3)")
open(os.path.join(RES, "thu_experiments.txt"), "w", encoding="utf-8").write(out.getvalue())

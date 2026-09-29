"""week02 구술 질문 대비 실험 (MuJoCo 3.x)
Q1. MJCF에서 body-joint 부모·자식 관계가 관절 좌표계를 어떻게 정하는가
Q2. inertial 태그를 생략하면 MuJoCo는 어떻게 처리하는가
사용법: python oral_experiments.py
"""
import numpy as np
import mujoco

np.set_printoptions(precision=4, suppress=True)

def load(xml):
    try:
        return mujoco.MjModel.from_xml_string(xml), None
    except ValueError as e:
        return None, str(e).strip().splitlines()[0]

print("=" * 70, "\nQ1. body pos/quat(부모 기준) → joint pos/axis(자식 body 기준)\n")
for quat, note in [("1 0 0 0", "회전 없음"), ("0.7071 0 0 0.7071", "z축 +90° 회전")]:
    for jpos in ["0 0 0", "0.1 0 0"]:
        xml = f"""<mujoco><worldbody>
          <body name="parent" pos="0 0 1">
            <geom type="box" size=".05 .05 .05"/>
            <body name="child" pos="0.3 0 0" quat="{quat}">
              <joint name="j" type="hinge" axis="0 1 0" pos="{jpos}"/>
              <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02"/>
            </body>
          </body></worldbody></mujoco>"""
        m, _ = load(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
        c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "child")
        print(f"child quat={note:12s} joint pos={jpos:8s} → child 원점(world)={d.xpos[c]}  "
              f"joint 축(world)={d.xaxis[0]}  joint 앵커(world)={d.xanchor[0]}")
print("\n→ 같은 axis=\"0 1 0\"이라도 child body의 quat에 따라 world 축이 바뀌고,"
      "\n  joint pos는 child body 좌표로 해석되어 회전 중심(앵커)만 옮긴다.")

print("\n" + "=" * 70, "\nQ2. inertial 생략 시 처리\n")
cases = {
    "A. geom만 있음 (inertiafromgeom=auto 기본)":
        '<mujoco><worldbody><body><joint type="hinge"/><geom type="box" size=".1 .05 .05"/></body></worldbody></mujoco>',
    "B. geom density=500 지정":
        '<mujoco><worldbody><body><joint type="hinge"/><geom type="box" size=".1 .05 .05" density="500"/></body></worldbody></mujoco>',
    "C. inertial 명시 + geom (auto → inertial 우선)":
        '<mujoco><worldbody><body><joint type="hinge"/><inertial pos="0 0 0" mass="2" diaginertia=".01 .01 .01"/>'
        '<geom type="box" size=".1 .05 .05"/></body></worldbody></mujoco>',
    "D. inertiafromgeom=true (inertial 무시하고 geom으로 덮어씀)":
        '<mujoco><compiler inertiafromgeom="true"/><worldbody><body><joint type="hinge"/>'
        '<inertial pos="0 0 0" mass="2" diaginertia=".01 .01 .01"/><geom type="box" size=".1 .05 .05"/></body></worldbody></mujoco>',
    "E. inertiafromgeom=false + inertial 없음":
        '<mujoco><compiler inertiafromgeom="false"/><worldbody><body><joint type="hinge"/>'
        '<geom type="box" size=".1 .05 .05"/></body></worldbody></mujoco>',
    "F. 움직이는 body에 geom도 inertial도 없음":
        '<mujoco><worldbody><body><joint type="hinge"/></body></worldbody></mujoco>',
    "G. visual 전용 geom(density=0)만 있음 (G1의 visual class와 같은 설정)":
        '<mujoco><worldbody><body><joint type="hinge"/><geom type="box" size=".1 .05 .05" density="0"/></body></worldbody></mujoco>',
    "H. F와 같지만 자식 body가 질량을 가짐":
        '<mujoco><worldbody><body><joint type="hinge"/><body pos="0.2 0 0"><joint type="hinge"/>'
        '<geom type="sphere" size=".03"/></body></body></worldbody></mujoco>',
    "I. 고정 body(joint 없음)에 geom/inertial 없음":
        '<mujoco><worldbody><body pos="0 0 1"><body><joint type="hinge"/><geom type="sphere" size=".03"/></body></body></worldbody></mujoco>',
}
vol_mass = 1000 * (0.2 * 0.1 * 0.1)
print(f"(참고) 0.2×0.1×0.1 m 상자, 기본 밀도 1000 kg/m³ → 질량 {vol_mass:.1f} kg\n")
for k, xml in cases.items():
    m, err = load(xml)
    if err:
        print(f"{k}\n    컴파일 오류: {err}\n")
    else:
        print(f"{k}\n    body1 mass={m.body_mass[1]:.3f} kg, diaginertia={m.body_inertia[1]}\n")

print("=" * 70, "\n(추가) URDF link에서 <inertial> 생략\n")
urdf = """<robot name="t"><link name="base"/>
  <link name="l1"><collision><geometry><box size="0.2 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"""
m, err = load(urdf)
print("URDF, inertial 없는 움직이는 link:", ("컴파일 오류: " + err) if err else f"mass={m.body_mass[-1]:.3f} kg")

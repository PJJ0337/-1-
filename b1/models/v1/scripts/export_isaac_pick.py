# week09 박종진 — Isaac 이식 준비: 집기 scene(그리퍼 손가락 · 테이블 · 물체)을 Isaac Lab 쪽 파일로 내보냄
#   MuJoCo는 생성 MJCF를 손대지 않고 메모리에서 손가락을 덧붙임(scenarios/pick_env.py). Isaac은 URDF 임포트라 같은 손가락을 URDF 사본에 넣어야 함
#   → 원본 urdf/b1_v1_isaac.urdf 는 그대로 두고, 손가락 2개(prismatic)를 덧붙인 사본 urdf/b1_v1_isaac_pick.urdf 를 생성(값 = pick_env.GRIPPER 한 곳)
#   → isaac/b1_pick_cfg.py : B1_PICK_CFG(ArticulationCfg — B1_V1_CFG + 그리퍼 액추에이터) · 테이블 · 물체 2종 RigidObjectCfg · 접촉 센서 cfg
#   손가락 대칭: MuJoCo equality(joint) → Isaac은 두 손가락에 같은 위치 목표(액션 항에서 1값 → 2관절). URDF <mimic>은 쓰지 않음(⚠ PhysX mimic 지원 버전 차이)
#   실행: python b1/models/v1/scripts/export_isaac_pick.py   → 생성 파일 2개 + compile 확인
import os,sys,re,json
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); ROOT=os.path.dirname(os.path.dirname(os.path.dirname(V1)))
sys.path.insert(0,os.path.join(ROOT,"scenarios")); import pick_env as P

G,C,T,O,S,GO=P.GRIPPER,P.CONTACT,P.TABLE,P.OBJ,P.SPAWN,P.GOAL
hx,hy,hz=G["finger_half"]

def finger_urdf():
    s="\n  <!-- week09 export_isaac_pick.py: 임시 평행 2지 손가락 (가정값 S15 = scenarios/pick_env.py GRIPPER). left_ee site 위치(gripper_link 아래 0.1 m)에 붙임 -->\n"
    for nm,sg in (("l",1),("r",-1)):
        s+=f'''  <joint name="left_finger_{nm}_joint" type="prismatic">
    <parent link="left_gripper_link"/>
    <child link="left_finger_{nm}"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="{sg} 0 0"/>
    <limit lower="0" upper="{G['stroke']}" effort="{G['grip_force']:g}" velocity="0.2"/>
    <dynamics damping="{G['damping']:g}" friction="0"/>
  </joint>
  <link name="left_finger_{nm}">
    <inertial>
      <origin xyz="{sg*hx:g} 0 {-hz:g}" rpy="0 0 0"/>
      <mass value="{G['finger_mass']:g}"/>
      <inertia ixx="2e-05" iyy="2e-05" izz="5e-06" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual name="left_pad_{nm}_visual">
      <origin xyz="{sg*hx:g} 0 {-hz:g}"/>
      <geometry><box size="{2*hx:g} {2*hy:g} {2*hz:g}"/></geometry>
    </visual>
    <collision name="left_pad_{nm}">
      <origin xyz="{sg*hx:g} 0 {-hz:g}"/>
      <geometry><box size="{2*hx:g} {2*hy:g} {2*hz:g}"/></geometry>
    </collision>
  </link>
'''
    return s

src=open(os.path.join(V1,"urdf","b1_v1_isaac.urdf"),encoding="utf-8").read()
assert src.count("</robot>")==1 and 'name="left_finger' not in src
dst=src.replace("</robot>",finger_urdf()+"</robot>")
dst=re.sub(r'<robot name="([^"]+)"',r'<robot name="\1_pick"',dst,count=1)
open(os.path.join(V1,"urdf","b1_v1_isaac_pick.urdf"),"w",encoding="utf-8").write(dst)

cx=T["edge_x"]+T["depth"]/2; cy=(T["y"][0]+T["y"][1])/2; mu=lambda k:C[k][0]
cfg=f'''# 자동 생성: b1/models/v1/scripts/export_isaac_pick.py (week09, 박종진) — 집기 scene Isaac Lab cfg 초안. 실습 PC 버전에서 import 경로 · 인자 이름 확인(⚠)
# 값 출처: scenarios/pick_env.py (GRIPPER · CONTACT · TABLE · OBJ — 가정값 표 S15 · S16 · S17). 로봇 본체 = b1_v1_cfg.B1_V1_CFG 그대로(팔 · 베이스 게인 week06 함태훈)
# 환경 전체 매핑(Observation · Action · Event · Termination · Reward) = b1/docs/isaaclab_env_spec.md
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg
from .b1_v1_cfg import B1_V1_CFG   # ⚠ 패키지로 넣을 때 상대 import, 단독 실행이면 from b1_v1_cfg import

HERE = os.path.dirname(os.path.abspath(__file__))

# 로봇: 원본 cfg + 손가락 URDF 사본 + 그리퍼 액추에이터
#   MuJoCo: position kp {G["kp"]:g}, forcerange ±{G["grip_force"]:g} N(손가락당), joint damping {G["damping"]:g}, armature 0.01 → Isaac: stiffness = kp, damping = joint damping, effort_limit_sim = 파지력
B1_PICK_CFG = B1_V1_CFG.replace(
    spawn=B1_V1_CFG.spawn.replace(asset_path=os.path.join(HERE, "..", "urdf", "b1_v1_isaac_pick.urdf")),
    actuators={{**B1_V1_CFG.actuators,
        "gripper": ImplicitActuatorCfg(joint_names_expr=["left_finger_[lr]_joint"], stiffness={G["kp"]:g}, damping={G["damping"]:g},
                                       effort_limit_sim={G["grip_force"]:g}, velocity_limit_sim=0.2, armature=0.01),
    }},
)
B1_PICK_CFG.init_state.joint_pos = {{".*": 0.0, "left_finger_[lr]_joint": {G["stroke"]}}}   # 그리퍼 벌림으로 시작 (MuJoCo reset과 같음)

# 마찰: MuJoCo friction(미끄럼 · 비틀림 · 구름) 첫 값만 PhysX static = dynamic 으로. condim 4(비틀림 포함) · solref · solimp 는 PhysX에 대응 항 없음 ⚠
#       MuJoCo 접촉 마찰 = 두 geom 중 큰 값(max) → PhysX friction_combine_mode="max" 로 맞춤
def _mat(mu):
    return sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, restitution=0.0, friction_combine_mode="max")

TABLE_CFG = RigidObjectCfg(
    prim_path="{{ENV_REGEX_NS}}/Table",
    spawn=sim_utils.CuboidCfg(size=({T["depth"]:g}, {T["y"][1]-T["y"][0]:g}, {T["top"]:g}),
                              rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                              collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat({mu("table_friction"):g})),
    init_state=RigidObjectCfg.InitialStateCfg(pos=({cx:g}, {cy:g}, {T["top"]/2:g})),
)
# 물체: 질량은 리셋 이벤트에서 0.2 ~ 3 kg (로그 균등) — 여기 값은 기본값
OBJ_BOX_CFG = RigidObjectCfg(
    prim_path="{{ENV_REGEX_NS}}/ObjBox",
    spawn=sim_utils.CuboidCfg(size=({2*O["box_half"]:g}, {2*O["box_half"]:g}, {2*O["box_half"]:g}), mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                              rigid_props=sim_utils.RigidBodyPropertiesCfg(), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat({mu("obj_friction"):g})),
    init_state=RigidObjectCfg.InitialStateCfg(pos=({T["edge_x"]+0.14:g}, 0.15, {T["top"]+O["box_half"]:g})),
)
OBJ_CYL_CFG = RigidObjectCfg(
    prim_path="{{ENV_REGEX_NS}}/ObjCyl",
    spawn=sim_utils.CylinderCfg(radius={O["cyl_r"]:g}, height={2*O["cyl_half"]:g}, axis="Z", mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                                rigid_props=sim_utils.RigidBodyPropertiesCfg(), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat({mu("obj_friction"):g})),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(-1.5, -1.5, {O["cyl_half"]:g})),   # 과제 물체가 아니면 MuJoCo처럼 바닥 멀리
)
# 손가락 패드 재질: 위 URDF 임포트 후 left_pad_* collision prim 에 _mat({mu("pad_friction"):g}) 적용 (⚠ UrdfFileCfg 에는 링크별 재질 인자가 없음 → 이벤트 randomize_rigid_body_material(body_names="left_finger_.*") 로 지정)

# 접촉 센서: 실패 판정(테이블 충돌 = 손가락 외 로봇 링크 ↔ 테이블), 파지 판정(패드 ↔ 물체)
ROBOT_TABLE_CONTACT_CFG = ContactSensorCfg(prim_path="{{ENV_REGEX_NS}}/Robot/(base_.*|left_(shoulder|elbow|gripper).*)", filter_prim_paths_expr=["{{ENV_REGEX_NS}}/Table"], history_length=1)
PAD_OBJ_CONTACT_CFG = ContactSensorCfg(prim_path="{{ENV_REGEX_NS}}/Robot/left_finger_.*", filter_prim_paths_expr=["{{ENV_REGEX_NS}}/ObjBox", "{{ENV_REGEX_NS}}/ObjCyl"], history_length=1)

# 시뮬: MuJoCo dt 0.5 ms 는 MuJoCo soft contact 수치 안정 조건(3 kg 원기둥 발산). PhysX TGS 는 따로 정함 → 시작값 dt 2 ms · decimation 10 (제어 50 Hz), 그리퍼 미끄러짐 표(gripper_slip.md) 재현으로 확인 ⚠
SIM_DT = 0.002
DECIMATION = 10
EPISODE_LENGTH_S = 34.0   # policies/eval_bc.py T_MAX (전문가 최장 28.6 s × 1.2)
GOAL_POS = ({T["edge_x"]+GO["depth"]:g}, {GO["y"]:g}, {T["top"]:g})   # 목표 원 중심(world), 반지름 {GO["tol"]:g} m
SPAWN = dict(x=({T["edge_x"]+S["depth"][0]:g}, {T["edge_x"]+S["depth"][1]:g}), y={tuple(S["y"])}, yaw_deg={tuple(S["yaw_deg"])}, mass_kg={tuple(O["mass"])})
'''
open(os.path.join(V1,"isaac","b1_pick_cfg.py"),"w",encoding="utf-8").write(cfg)
compile(cfg,"b1_pick_cfg.py","exec")
# URDF 점검: 손가락 관절 2개 · 링크 2개 추가, 나머지 줄은 원본과 같음
import xml.etree.ElementTree as ET
r=ET.fromstring(dst); J=[j.get("name") for j in r.findall("joint")]; L=[l.get("name") for l in r.findall("link")]
assert "left_finger_l_joint" in J and "left_finger_r_joint" in J and len(J)==len(ET.fromstring(src).findall("joint"))+2
print("urdf ok: joints",len(J),"links",len(L),"| cfg ok")

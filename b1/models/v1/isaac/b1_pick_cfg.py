# 자동 생성: b1/models/v1/scripts/export_isaac_pick.py (week09, 박종진) — 집기 scene Isaac Lab cfg 초안. 실습 PC 버전에서 import 경로 · 인자 이름 확인(⚠)
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
#   MuJoCo: position kp 3000, forcerange ±40 N(손가락당), joint damping 8, armature 0.01 → Isaac: stiffness = kp, damping = joint damping, effort_limit_sim = 파지력
B1_PICK_CFG = B1_V1_CFG.replace(
    spawn=B1_V1_CFG.spawn.replace(asset_path=os.path.join(HERE, "..", "urdf", "b1_v1_isaac_pick.urdf")),
    actuators={**B1_V1_CFG.actuators,
        "gripper": ImplicitActuatorCfg(joint_names_expr=["left_finger_[lr]_joint"], stiffness=3000, damping=8,
                                       effort_limit_sim=40, velocity_limit_sim=0.2, armature=0.01),
    },
)
B1_PICK_CFG.init_state.joint_pos = {".*": 0.0, "left_finger_[lr]_joint": 0.055}   # 그리퍼 벌림으로 시작 (MuJoCo reset과 같음)

# 마찰: MuJoCo friction(미끄럼 · 비틀림 · 구름) 첫 값만 PhysX static = dynamic 으로. condim 4(비틀림 포함) · solref · solimp 는 PhysX에 대응 항 없음 ⚠
#       MuJoCo 접촉 마찰 = 두 geom 중 큰 값(max) → PhysX friction_combine_mode="max" 로 맞춤
def _mat(mu):
    return sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu, restitution=0.0, friction_combine_mode="max")

TABLE_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Table",
    spawn=sim_utils.CuboidCfg(size=(0.6, 1.8, 0.7),
                              rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                              collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat(0.8)),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.75, 0, 0.35)),
)
# 물체: 질량은 리셋 이벤트에서 0.2 ~ 3 kg (로그 균등) — 여기 값은 기본값
OBJ_BOX_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/ObjBox",
    spawn=sim_utils.CuboidCfg(size=(0.06, 0.06, 0.06), mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                              rigid_props=sim_utils.RigidBodyPropertiesCfg(), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat(1)),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(0.59, 0.15, 0.73)),
)
OBJ_CYL_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/ObjCyl",
    spawn=sim_utils.CylinderCfg(radius=0.028, height=0.065, axis="Z", mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
                                rigid_props=sim_utils.RigidBodyPropertiesCfg(), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=_mat(1)),
    init_state=RigidObjectCfg.InitialStateCfg(pos=(-1.5, -1.5, 0.0325)),   # 과제 물체가 아니면 MuJoCo처럼 바닥 멀리
)
# 손가락 패드 재질: 위 URDF 임포트 후 left_pad_* collision prim 에 _mat(1) 적용 (⚠ UrdfFileCfg 에는 링크별 재질 인자가 없음 → 이벤트 randomize_rigid_body_material(body_names="left_finger_.*") 로 지정)

# 접촉 센서: 실패 판정(테이블 충돌 = 손가락 외 로봇 링크 ↔ 테이블), 파지 판정(패드 ↔ 물체)
ROBOT_TABLE_CONTACT_CFG = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/(base_.*|left_(shoulder|elbow|gripper).*)", filter_prim_paths_expr=["{ENV_REGEX_NS}/Table"], history_length=1)
PAD_OBJ_CONTACT_CFG = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/left_finger_.*", filter_prim_paths_expr=["{ENV_REGEX_NS}/ObjBox", "{ENV_REGEX_NS}/ObjCyl"], history_length=1)

# 시뮬: MuJoCo dt 0.5 ms 는 MuJoCo soft contact 수치 안정 조건(3 kg 원기둥 발산). PhysX TGS 는 따로 정함 → 시작값 dt 2 ms · decimation 10 (제어 50 Hz), 그리퍼 미끄러짐 표(gripper_slip.md) 재현으로 확인 ⚠
SIM_DT = 0.002
DECIMATION = 10
EPISODE_LENGTH_S = 34.0   # policies/eval_bc.py T_MAX (전문가 최장 28.6 s × 1.2)
GOAL_POS = (0.6, -0.3, 0.7)   # 목표 원 중심(world), 반지름 0.04 m
SPAWN = dict(x=(0.51, 0.67), y=(0.0, 0.35), yaw_deg=(-45, 45), mass_kg=(0.2, 3.0))

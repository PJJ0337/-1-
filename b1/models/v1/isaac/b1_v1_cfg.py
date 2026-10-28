# 자동 생성: b1/models/v1/scripts/export_isaac.py (week06, 박종진) — Isaac Lab ArticulationCfg 초안. 실습 PC 버전에서 import 경로 · 인자 이름 확인(⚠)
# 값 출처: mjcf/b1.xml 컴파일 결과(A2 case "a2_doc"), urdf/b1_v1_isaac.urdf. 팔 stiffness · damping = 함태훈 week06 고정 게인(inputs/isaac_arm_gains.json ← sim/w06/docs/actuator_params.md)
import os
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

HERE = os.path.dirname(os.path.abspath(__file__))
B1_V1_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=os.path.join(HERE, "..", "urdf", "b1_v1_isaac.urdf"),
        fix_base=True,                      # 루트 odom 고정, 베이스는 prismatic x · y + continuous yaw 로 이동
        merge_fixed_joints=False,           # gripper_link(ee) · base_plate_link(IMU) 보존
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            drive_type="force", target_type="position",
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=400, damping=31.0),   # 임포트 기본값, 실제 게인은 아래 actuators가 덮어씀
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),   # 중력보상은 관절 중력 항 effort 피드포워드로 (토크 예산에 중력 포함, 함태훈 actuator_params.md 2절)
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), joint_pos={".*": 0.0}),   # 영점 = 팔 내림 · 팔꿈치 전방 굽힘 방향
    actuators={
        "base": ImplicitActuatorCfg(joint_names_expr=["base_.*"], stiffness=0.0,
                                    damping={"base_[xy]_joint": 200, "base_yaw_joint": 50},
                                    effort_limit_sim={"base_[xy]_joint": 2000, "base_yaw_joint": 500}),
        "shoulder": ImplicitActuatorCfg(joint_names_expr=[".*_shoulder_(pitch|roll|yaw)_joint"],
                                        stiffness={".*_shoulder_pitch_joint": 400, ".*_shoulder_roll_joint": 350, ".*_shoulder_yaw_joint": 300},
                                        damping={".*_shoulder_pitch_joint": 31.0, ".*_shoulder_roll_joint": 27.5, ".*_shoulder_yaw_joint": 15.0},
                                        effort_limit_sim=381.7, velocity_limit_sim=3,
                                        armature=0.219, friction=0.0),
        "elbow": ImplicitActuatorCfg(joint_names_expr=[".*_elbow_joint"], stiffness=300, damping=11.0,
                                     effort_limit_sim=181.5, velocity_limit_sim=3,
                                     armature=0.098, friction=0.0),
    },
)
# 끝점: FrameTransformerCfg(target prim ".../{left,right}_gripper_link", offset pos (0, 0, -0.1)) · IMU: ImuCfg(prim ".../base_plate_link")

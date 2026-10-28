# Isaac 임포트 패키지 점검 + 대응표 생성 (week06 커리큘럼 금 · 박종진 개별 과제)
#   1) 패키지 점검: URDF 메시 경로 = 상대 경로(../meshes/visual|collision, package:// 없음) · 파일 존재 · m 단위 · 면 수
#   2) docs/import_settings.md : MuJoCo 설정 ↔ Isaac Sim URDF Importer 옵션 ↔ Isaac Lab ArticulationCfg 대응표 (수치는 컴파일된 b1.xml에서)
#   3) isaac/b1_v1_cfg.py      : Isaac Lab ArticulationCfg 초안 (실습 PC 첫날 그대로 붙여 넣고 확인할 값)
# 사용: python export_isaac.py
import os,sys,json,re,glob,numpy as np,mujoco,trimesh,xml.etree.ElementTree as ET
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); MODELS=os.path.dirname(V1); sys.path.insert(0,HERE); from v1_mjio import load_model
A=json.load(open(os.path.join(MODELS,"assumptions_b1.json"),encoding="utf-8")); P=json.load(open(os.path.join(V1,"actuator_params.json"),encoding="utf-8"))
A2=json.load(open(os.path.join(V1,"inputs","a2_actuators.json"),encoding="utf-8")); CASE=P.get("a2",{}).get("case","none")
GI=json.load(open(os.path.join(V1,"inputs","isaac_arm_gains.json"),encoding="utf-8"))["joints"]   # (10/28) 함태훈 week06 고정 kv 게인 → Isaac 팔 게인
m=load_model(os.path.join(V1,"mjcf","scene.xml")); ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
# ---- 1) 패키지 점검
chk=[]
for U in ("b1_v1.urdf","b1_v1_isaac.urdf"):
    up=os.path.join(V1,"urdf",U); r=ET.parse(up).getroot(); files=[e.get("filename") for e in r.iter("mesh")]
    rel=all(f.startswith(("../meshes/visual/","../meshes/collision/")) for f in files); pk=any("package://" in f for f in files)
    ex=all(os.path.exists(os.path.normpath(os.path.join(V1,"urdf",f))) for f in files)
    chk.append((U,len(files),rel and not pk,ex,[j.get("type") for j in r.findall("joint")][:3]))
MI={}
for kind in ("visual","collision"):
    for f in sorted(glob.glob(os.path.join(V1,"meshes",kind,"*.stl"))):
        t=trimesh.load(f); MI[f"{kind}/{os.path.basename(f)}"]=dict(faces=int(len(t.faces)),extent_m=round(float(max(t.extents)),4),watertight=bool(t.is_watertight),convex=bool(t.is_convex))
# ---- 컴파일 수치
act={}
for i in range(m.nu):
    n=m.actuator(i).name; j=m.actuator_trnid[i,0]; dof=m.jnt_dofadr[j]
    act[n]=dict(joint=m.joint(j).name,kp=float(m.actuator_gainprm[i][0]),kv=round(float(-m.actuator_biasprm[i][2]),3),force=float(m.actuator_forcerange[i][1]),
                armature=float(m.dof_armature[dof]),damping=float(m.dof_damping[dof]),frictionloss=float(m.dof_frictionloss[dof]),range=[round(float(x),3) for x in m.jnt_range[j]])
ri=ET.parse(os.path.join(V1,"urdf","b1_v1_isaac.urdf")).getroot()
vel={j.get("name"):float(j.find("limit").get("velocity")) for j in ri.findall("joint") if j.find("limit") is not None}
eff={j.get("name"):float(j.find("limit").get("effort")) for j in ri.findall("joint") if j.find("limit") is not None}
# ---- 2) import_settings.md
a=act; sp,se=a["left_shoulder_pitch"],a["left_elbow"]
L=[f"# Isaac 임포트 대응표 — MuJoCo ↔ Isaac Sim URDF Importer ↔ Isaac Lab (자동 생성: export_isaac.py, week06)","",
 "실습 PC 배정 첫날 이 표대로 임포트하고, 마지막 절 「첫날 할 일」로 MuJoCo 검증 ② · ③을 재현해 수치를 비교한다. 수치는 컴파일된 `mjcf/b1.xml`(A2 case `"+CASE+"`)에서 읽음.",
 "기준 문서: Isaac Sim 5.1 URDF Importer · Isaac Lab Import New Asset / Actuators / ArticulationCfg. **Isaac 쪽 기본값 · 옵션 이름은 버전마다 달라 실습 PC 버전에서 확인할 것(⚠ 표시)**","",
 "## 1. 임포트할 파일","",
 "| 파일 | 용도 | 점검 |","|---|---|---|"]
for U,n,rel,ex,jt in chk:
    L.append(f'| `urdf/{U}` | {"MuJoCo 원본(planar) — Isaac 미지원" if U=="b1_v1.urdf" else "**Isaac 임포트용** (planar → prismatic x · prismatic y · continuous z + 가상 링크 2)"} | 메시 {n}개 상대 경로 {"OK" if rel else "NG"} · 파일 존재 {"OK" if ex else "NG"}, 베이스 관절 {jt} |')
L+=[f'| `meshes/visual/*.stl` ({sum(1 for k in MI if k.startswith("visual"))}개) | 표시용 (2만 면 이하 단순화본) | 면 수 {min(v["faces"] for k,v in MI.items() if k.startswith("visual")):,} ~ {max(v["faces"] for k,v in MI.items() if k.startswith("visual")):,}, 최대 길이 {max(v["extent_m"] for v in MI.values()):.3f} m (m 단위) |',
    f'| `meshes/collision/*.stl` ({sum(1 for k in MI if k.startswith("collision"))}개) | 충돌용 (볼록 껍질) | 면 수 {min(v["faces"] for k,v in MI.items() if k.startswith("collision"))} ~ {max(v["faces"] for k,v in MI.items() if k.startswith("collision"))}, 모두 convex = {all(v["convex"] for k,v in MI.items() if k.startswith("collision"))} |',
    "| 상판 · 하체 · 몸통 box · 그리퍼 | URDF `<box>` 기본 도형 | 상판 메시(모바일로봇 상판 STEP)는 **넣지 않음** — 실측 치수 box 0.576 × 0.472 × 0.012 m · 3.69 kg로 충분(질량 · 충돌), 표시용 메시는 A팀 확정 후 |",
    "","## 2. 설정 대응표","",
    "| 항목 | MuJoCo (우리 설정) | Isaac Sim URDF Importer | Isaac Lab | 비고 |","|---|---|---|---|---|",
    "| 고정 관절 병합 | `<compiler fusestatic=\"false\">` (base_plate_link · torso · cad_frame · tool0 · gripper_link 보존) | Merge Fixed Joints **끔** | `UrdfConverterCfg(merge_fixed_joints=False)` | 켜면 gripper_link · base_plate_link가 사라져 ee · IMU 기준 프레임을 잃음. Isaac Lab 문서는 대부분 True 권장 → 우리는 프레임 보존 우선. 성능 문제 시 cad_frame · tool0만 병합 |",
    "| 베이스 | planar 1개 (원본) → slide · slide · hinge 3-DOF, velocity 액추에이터 | Isaac용 사본 사용, **Static base(Fix Base) 켬** — 루트 `odom`을 world에 고정하고 3관절로 이동 | `fix_base=True` | Moveable base로 두면 6-DOF 자유 + 3관절이 중복. 옴니휠 미모델링(알려진 한계) |",
    f'| 베이스 구동 | velocity kv vx · vy {a["base_vx"]["kv"]:g}, wz {a["base_wz"]["kv"]:g}, forcerange ±{a["base_vx"]["force"]:g} / ±{a["base_wz"]["force"]:g} | Joint Drive: Target = Velocity, Stiffness 0, Damping = kv | `ImplicitActuatorCfg(joint_names_expr=["base_.*"], stiffness=0.0, damping={{"base_[xy]_joint": {a["base_vx"]["kv"]:g}, "base_yaw_joint": {a["base_wz"]["kv"]:g}}})` | 속도 제어 = D 게인만 |',
    f'| 팔 구동 | position kp {sp["kp"]:g}, dampratio 1 → kv(컴파일) pitch {a["left_shoulder_pitch"]["kv"]} · roll {a["left_shoulder_roll"]["kv"]} · yaw {a["left_shoulder_yaw"]["kv"]} · elbow {a["left_elbow"]["kv"]} (week05 방식, MuJoCo 빌드 현재값) | Joint Drive: Drive Type **Force**, Target **Position**, Joint Configuration = **Stiffness** 모드로 kp · kv 직접 입력 | **함태훈 week06 고정 게인** `ImplicitActuatorCfg(stiffness, damping)`: pitch {GI["shoulder_pitch"]["stiffness"]} · {GI["shoulder_pitch"]["damping"]} / roll {GI["shoulder_roll"]["stiffness"]} · {GI["shoulder_roll"]["damping"]} / yaw {GI["shoulder_yaw"]["stiffness"]} · {GI["shoulder_yaw"]["damping"]} / elbow {GI["elbow"]["stiffness"]} · {GI["elbow"]["damping"]} (`inputs/isaac_arm_gains.json` ← 함태훈 `sim/w06/docs/actuator_params.md`) | Isaac에는 dampratio가 없어 고정 kv로 튜닝(두 case에서 오버슈트 ≤ 1.5 %, 정착 ≤ 0.29 s). Natural Frequency 모드는 Kp = m·ωn², Kd = 2m·ζ·ωn(m = 관절 등가관성). ⚠ stiffness 단위 = N·m/rad, MuJoCo joint damping 0.5는 PhysX에 별도 항이 없어 damping에 더하는지 확인 |',
    f'| 토크 한계 | forcerange ±{sp["force"]:g}(어깨) · ±{se["force"]:g}(팔꿈치) = A2 피크 | URDF `<limit effort>` = 같은 값 | `effort_limit_sim` (⚠ 2.x 이전 이름 `effort_limit`) | PDR 판정은 정격(어깨 TQ 118.8 · 팔꿈치 RI85 72.6, A2 10/19). 정격으로 묶어 보려면 effort만 바꿔 재빌드 |',
    f'| 속도 한계 | (MuJoCo는 속도 한계 없음 — 궤적에서 관리) | URDF `<limit velocity>` {vel.get("left_shoulder_pitch_joint")} rad/s | `velocity_limit_sim` | A2 TQ star-serial 무부하 3.50 rad/s 이하로 둠 |',
    f'| 반사 관성 | `armature` 어깨 {sp["armature"]} · 팔꿈치 {se["armature"]} kg·m² (A2 J_rotor × i², 로터만) | URDF에 항목 없음 → 임포트 후 설정 | `ImplicitActuatorCfg(armature=…)` | 기어열 관성 TBD(A2) → 과소 추정 |',
    f'| 관절 감쇠 · 마찰 | joint damping {sp["damping"]} · frictionloss {sp["frictionloss"]} (C4 가정) | URDF `<dynamics damping friction>` 로 들어옴 ⚠ 버전에 따라 drive damping에 더해지는지 확인 | `friction` (· 필요 시 `viscous_friction`) | A2 마찰 · 백드라이브 TBD(다이나모 시험 후) |',
    "| 중력보상 | 팔 body `gravcomp=1` + joint `actuatorgravcomp=true` (week06 기본 on) — 보상 토크가 액추에이터 쪽, forcerange 안에서 | 대응 옵션 없음 | **관절 중력 항을 effort 피드포워드로**(권장, 함태훈 10/28) — `disable_gravity=True`는 중력이 토크 예산에서 빠짐 | actuatorgravcomp에 해당하는 API 없음 ⚠ 실습 PC에서 중력 항 읽는 API 확인 |",
    "| 자기 충돌 | 부모-자식 자동 제외, 추가 exclude 0 (검증 ④: 범위 안 3,770자세 접촉 0) | Allow self-collision **켬** (인접 링크는 PhysX가 자동 필터) | `ArticulationRootPropertiesCfg(enabled_self_collisions=True)` | ⚠ 문서는 기본 끔 권장 — 우리는 ④로 범위 안 무충돌 확인했으므로 켜서 팔-몸통 충돌을 잡음. 폭주 시 끄고 원인 링크 기록 |",
    "| 충돌 형상 | collision STL = 볼록 껍질 (MuJoCo는 메시를 항상 convex hull) | Collider Type **Convex Hull**, Collision From Visuals 끔 | `UrdfConverterCfg(collider_type=\"convex_hull\")` ⚠ | 이미 볼록 → 분해 불필요, MuJoCo와 같은 형상 |",
    "| 질량 · 관성 | URDF inertial 그대로, `balanceinertia=false` | Import Inertia Tensor 켬, Default Density 0(미사용) | 기본 | 모든 링크 질량 지정(가상 링크 1 g) |",
    "| 메시 경로 | `strippath=false`, URDF `../meshes/visual/x.stl` → MJCF `meshdir=../meshes/` + `visual/x.stl` | 상대 경로 그대로 해석 | `asset_path` = URDF 파일 | 한글 · 공백 경로 피할 것(Isaac은 특수문자를 `_`로 바꿈) — 실습 PC에서는 영문 경로에 복사 |",
    "| 끝점 · IMU | site `left_ee` · `right_ee`(gripper_link에서 (0, 0, −0.10)), `base_imu`(base_plate_link 중심) | site 개념 없음 | `FrameTransformerCfg`(prim = {side}_gripper_link, offset pos (0, 0, −0.10)) · `ImuCfg`(prim = base_plate_link) | merge_fixed_joints=False 라서 두 링크가 남아 있음 |",
    "| 센서 | jointpos · jointvel · actuatorfrc · jointactuatorfrc × 8, gyro · accelerometer, framepos · framequat × 2 (38) | — | `robot.data.joint_pos` · `joint_vel` · `applied_torque`, ImuCfg, FrameTransformerCfg | |",
    "| 적분 · 시간 | timestep 0.002 s, implicitfast | — | `SimulationCfg(dt=0.002)` 권장(같은 dt로 비교), 제어 decimation은 태스크에서 | PhysX TGS, solver iteration 기본 |",
    "| 관절 이름 · 순서 | ICD `{side}_{부위}_{운동}_joint`, 액추에이터 `{side}_{…}` | URDF 이름 그대로 | `joint_names_expr` 정규식 | Isaac 관절 순서는 BFS라 MuJoCo(qpos 순)와 다를 수 있음 → 이름으로 매핑 |",
    "","## 3. Isaac Lab Manager-Based 구조 ↔ 이번 주 MuJoCo 코드","",
    "| Manager | 의미 | 우리 MuJoCo 코드에서 해당 부분 |","|---|---|---|",
    "| Scene | 로봇 · 바닥 · 센서 배치 | `mjcf/scene.xml` (include b1.xml + 바닥 · 조명) |",
    "| Action | 정책 출력 → 액추에이터 명령 | `d.ctrl` = 팔 관절 목표각(position) + 베이스 속도(velocity) → `JointPositionActionCfg` · 베이스는 `JointVelocityActionCfg` |",
    "| Observation | 정책 입력 | 센서 jointpos · jointvel · ee framepos, IMU — `ik.py`의 목표 위치(base 기준)를 명령으로 추가 |",
    "| Reward | 학습 목표 | IK 결과표의 위치 오차 · 그리퍼 축 오차, 토크 예산(정격 대비) 패널티 |",
    "| Termination | 에피소드 종료 | 자기 충돌(검증 ④), 관절 한계 접촉, 목표 도달 |",
    "| Event | 리셋 · 무작위화 | 작업공간 샘플에서 목표 뽑기(`results/workspace_points.npz`), 페이로드 0 / 0.5 / 1.0 kg 무작위 |",
    "","## 4. 실습 PC 첫날 할 일 (순서)","",
    "1. 저장소 clone → `b1/models/v1` 을 영문 경로에 복사, `python b1/models/v1/scripts/build_v1.py` 로 재생성(값 확인)",
    "2. Isaac Sim URDF Importer로 `urdf/b1_v1_isaac.urdf` 임포트 — 2절 옵션(Fix Base 켬 · Merge Fixed Joints 끔 · Convex Hull · Self-collision 켬 · Drive Force/Position/Stiffness)",
    f"3. USD 저장 후 링크 {len(ri.findall('link'))}개(odom · 가상 base_x/base_y 포함) · 관절 {len(ri.findall('joint'))}개(움직이는 관절 {sum(j.get('type')!='fixed' for j in ri.findall('joint'))}) · 총질량 75.10 kg 확인 (MuJoCo `validate_model.md` ① 과 비교)",
    "4. 검증 ② 재현: 팔 앞으로 수평(pitch −90°) 5 s 유지 — 중력보상 없음이면 처짐 ≈ τg/kp = 4.85°가 나와야 함",
    "5. 검증 ③ 재현: 같은 자세 정적 토크 pitch 25.40 N·m(0 kg) — `robot.data.applied_torque` 또는 `compute_gravity_compensation` 비교",
    "6. `isaac/b1_v1_cfg.py` 를 Isaac Lab 태스크에 넣고 IK 목표 10개(`results/ik_results.json`)를 관절 목표로 재생 → ee 위치 차 기록",
    "7. 3주차에서 미룬 USD 실습(인스턴서블 메시 · 머티리얼) 진행",
    "",f"## 5. 메시 목록 (면 수)","","| 파일 | 면 수 | 최대 길이 [m] | 볼록 |","|---|---|---|---|"]+[f'| {k} | {v["faces"]:,} | {v["extent_m"]} | {v["convex"]} |' for k,v in MI.items()]
os.makedirs(os.path.join(V1,"docs"),exist_ok=True); open(os.path.join(V1,"docs","import_settings.md"),"w",encoding="utf-8").write("\n".join(L)+"\n")
# ---- 3) Isaac Lab cfg 초안
kv={x:GI[x]["damping"] for x in ARM}; kp={x:GI[x]["stiffness"] for x in ARM}   # 팔 게인 = 함태훈 week06 고정 게인
cfg=f'''# 자동 생성: b1/models/v1/scripts/export_isaac.py (week06, 박종진) — Isaac Lab ArticulationCfg 초안. 실습 PC 버전에서 import 경로 · 인자 이름 확인(⚠)
# 값 출처: mjcf/b1.xml 컴파일 결과(A2 case "{CASE}"), urdf/b1_v1_isaac.urdf. 팔 stiffness · damping = 함태훈 week06 고정 게인(inputs/isaac_arm_gains.json ← sim/w06/docs/actuator_params.md)
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
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness={kp["shoulder_pitch"]:g}, damping={kv["shoulder_pitch"]}),   # 임포트 기본값, 실제 게인은 아래 actuators가 덮어씀
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),   # 중력보상은 관절 중력 항 effort 피드포워드로 (토크 예산에 중력 포함, 함태훈 actuator_params.md 2절)
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0), joint_pos={{".*": 0.0}}),   # 영점 = 팔 내림 · 팔꿈치 전방 굽힘 방향
    actuators={{
        "base": ImplicitActuatorCfg(joint_names_expr=["base_.*"], stiffness=0.0,
                                    damping={{"base_[xy]_joint": {act["base_vx"]["kv"]:g}, "base_yaw_joint": {act["base_wz"]["kv"]:g}}},
                                    effort_limit_sim={{"base_[xy]_joint": {act["base_vx"]["force"]:g}, "base_yaw_joint": {act["base_wz"]["force"]:g}}}),
        "shoulder": ImplicitActuatorCfg(joint_names_expr=[".*_shoulder_(pitch|roll|yaw)_joint"],
                                        stiffness={{".*_shoulder_pitch_joint": {kp["shoulder_pitch"]:g}, ".*_shoulder_roll_joint": {kp["shoulder_roll"]:g}, ".*_shoulder_yaw_joint": {kp["shoulder_yaw"]:g}}},
                                        damping={{".*_shoulder_pitch_joint": {kv["shoulder_pitch"]}, ".*_shoulder_roll_joint": {kv["shoulder_roll"]}, ".*_shoulder_yaw_joint": {kv["shoulder_yaw"]}}},
                                        effort_limit_sim={act["left_shoulder_pitch"]["force"]:g}, velocity_limit_sim={vel["left_shoulder_pitch_joint"]:g},
                                        armature={act["left_shoulder_pitch"]["armature"]}, friction={act["left_shoulder_pitch"]["frictionloss"]}),
        "elbow": ImplicitActuatorCfg(joint_names_expr=[".*_elbow_joint"], stiffness={kp["elbow"]:g}, damping={kv["elbow"]},
                                     effort_limit_sim={act["left_elbow"]["force"]:g}, velocity_limit_sim={vel["left_elbow_joint"]:g},
                                     armature={act["left_elbow"]["armature"]}, friction={act["left_elbow"]["frictionloss"]}),
    }},
)
# 끝점: FrameTransformerCfg(target prim ".../{{left,right}}_gripper_link", offset pos (0, 0, -{A["gripper"]["ee_offset"]})) · IMU: ImuCfg(prim ".../base_plate_link")
'''
os.makedirs(os.path.join(V1,"isaac"),exist_ok=True); open(os.path.join(V1,"isaac","b1_v1_cfg.py"),"w",encoding="utf-8").write(cfg)
compile(cfg,"b1_v1_cfg.py","exec")
json.dump(dict(urdf_check=[dict(file=U,n_mesh=n,relative=rel,exists=ex,base_joints=jt) for U,n,rel,ex,jt in chk],meshes=MI,actuators=act,a2_case=CASE),open(os.path.join(V1,"results","isaac_package_check.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
print("\n".join(L[7:12])); print("cfg ok")

# week08 박종진 — 접촉 집기 scene 생성 (테이블 · 물체 2종 · 임시 평행 2지 그리퍼)
#   생성된 b1.xml(빌드 산출물)은 디스크에서 고치지 않는다 → 메모리에서 b1.xml 문자열의 left_gripper_link 안에 손가락 2개를 덧붙이고,
#   scene.xml에 테이블 · 물체 · 목표 영역을 덧붙여 컴파일 (5주차 빌드 규약 유지)
#   그리퍼 · 접촉 값은 전부 가정값(GRIPPER · CONTACT) — 함태훈 week08 그리퍼 · 접촉 모델 받으면 이 두 dict만 교체
#   사용: from pick_env import build, Episode;  m = build(); ep = Episode(m, seed)
import os,sys
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
V1=os.path.join(ROOT,"b1","models","v1"); sys.path.insert(0,os.path.join(V1,"scripts"))
from v1_mjio import _assets

# ---- 가정값 (assumptions: 그리퍼 사양 미수령 · A팀 회신 시 교체) ----
GRIPPER=dict(stroke=0.055,          # 손가락 1개 이동 [m] (안쪽 면 0 ~ 0.055 → 최대 벌림 0.11 m)
             finger_half=(0.008,0.015,0.03),   # 손가락 box 반치수 x(두께) · y(폭) · z(길이) [m] → 길이 0.06 m
             finger_mass=0.2, grip_force=40.0,  # 손가락 0.05 kg이면 3 kg 물체와 질량비 60 → 들기 시작에서 수치 발산(8주차 발견) → 0.2 kg # 손가락당 수직력 상한 [N] (position 액추에이터 forcerange)
             kp=3000.0, damping=8.0,
             grasp_off=0.045)       # ee site(손바닥 아래면) → 파지 중심 거리 [m] (손바닥 ↔ 물체 윗면 여유 ≥ 15 mm)
CONTACT=dict(pad_friction=(1.0,0.005,0.0001), obj_friction=(1.0,0.005,0.0001), table_friction=(0.8,0.005,0.0001),
             condim=4, solref=(0.01,1.0), solimp=(0.95,0.99,0.001))
OPTION='timestep="0.0005" cone="elliptic" impratio="10"'   # 파지 크리프 방지(gripper_slip.md: 기본 pyramidal은 F ≫ F_min에서도 흘러내림) + 시간 간격 0.5 ms(2 ms면 2 ~ 3 kg 들기 · 놓기 발산, 1 ms도 3 kg 원기둥 들기에서 수치 진동 발산 — 함태훈 week08 7f434ae contact_sweep: dt 0.5 ms만 해결, 3 kg 20회 9 → 19/20)
TABLE=dict(edge_x=0.45, depth=0.6, y=(-0.9,0.9), top=0.70)   # 테이블 앞 가장자리 world x, 윗면 높이 (7주차 그림 · 작업공간 분석과 같은 0.70 m)
OBJ=dict(box_half=0.03, cyl_r=0.028, cyl_half=0.0325, mass=(0.2,3.0))
SPAWN=dict(depth=(0.06,0.22), y=(0.00,0.35), yaw_deg=(-45,45))   # 물체 초기 위치 무작위 범위(테이블 가장자리 기준 안쪽 깊이 · world y)
GOAL=dict(depth=0.15, y=-0.30, tol=0.04)                        # 놓을 목표 영역(원) 중심 · 반지름 [m]

def _f(v): return " ".join(f"{x:g}" for x in v)

def _fingers_xml():
    G,C=GRIPPER,CONTACT; hx,hy,hz=G["finger_half"]
    common=(f'friction="{_f(C["pad_friction"])}" condim="{C["condim"]}" solref="{_f(C["solref"])}" solimp="{_f(C["solimp"])}" '
            f'rgba="0.25 0.25 0.3 1"')
    s=""
    for nm,sg in (("l",1),("r",-1)):
        s+=(f'<body name="left_finger_{nm}" pos="0 0 -0.1">'
            f'<inertial pos="{sg*hx:g} 0 {-hz:g}" mass="{G["finger_mass"]}" diaginertia="2e-5 2e-5 5e-6"/>'
            f'<joint name="left_finger_{nm}_joint" type="slide" axis="{sg} 0 0" range="0 {G["stroke"]}" damping="{G["damping"]}" armature="0.01"/>'
            f'<geom name="left_pad_{nm}" type="box" size="{hx} {hy} {hz}" pos="{sg*hx:g} 0 {-hz:g}" {common}/></body>')
    return s

def build(cam=False):
    """집기 scene 모델. cam=True면 오프스크린 렌더용 카메라 2개(정면 위 · 손목 시점 아님) 포함"""
    sc=os.path.join(V1,"mjcf","scene.xml"); a=_assets(sc)
    b1=a["b1.xml"].decode("utf-8")
    key='<site name="left_ee" pos="0 0 -0.1"'
    i=b1.index(key); b1=b1[:i]+_fingers_xml()+b1[i:]
    # 손가락끼리 · 손가락 ↔ 손바닥 상자 충돌 제외 + 손가락 대칭(equality) + 액추에이터 2개(같은 명령, 손가락당 힘 상한)
    G=GRIPPER
    extra=(f'<contact><exclude body1="left_finger_l" body2="left_finger_r"/></contact>'
           f'<equality><joint joint1="left_finger_r_joint" joint2="left_finger_l_joint" polycoef="0 1 0 0 0" solref="0.005 1"/></equality>')
    b1=b1.replace("<actuator>",extra+"<actuator>",1)
    acts="".join(f'<position name="left_finger_{n}" joint="left_finger_{n}_joint" kp="{G["kp"]}" ctrlrange="0 {G["stroke"]}" forcerange="{-G["grip_force"]} {G["grip_force"]}"/>' for n in ("l","r"))
    b1=b1.replace("</actuator>",acts+"</actuator>",1)
    a["b1.xml"]=b1.encode("utf-8")
    T,C,O=TABLE,CONTACT,OBJ; cx=T["edge_x"]+T["depth"]/2; cy=(T["y"][0]+T["y"][1])/2
    cc=f'condim="{C["condim"]}" solref="{_f(C["solref"])}" solimp="{_f(C["solimp"])}"'
    world=(f'<geom name="table" type="box" pos="{cx:g} {cy:g} {T["top"]/2:g}" size="{T["depth"]/2:g} {(T["y"][1]-T["y"][0])/2:g} {T["top"]/2:g}" '
           f'friction="{_f(C["table_friction"])}" {cc} rgba="0.72 0.55 0.38 1"/>'
           f'<site name="goal" type="cylinder" pos="{T["edge_x"]+GOAL["depth"]:g} {GOAL["y"]:g} {T["top"]+0.001:g}" size="{GOAL["tol"]:g} 0.001" rgba="0.1 0.8 0.2 0.5"/>'
           f'<body name="obj_box" pos="2 2 0.03"><freejoint name="obj_box_free"/>'
           f'<geom name="obj_box" type="box" size="{O["box_half"]} {O["box_half"]} {O["box_half"]}" mass="1" friction="{_f(C["obj_friction"])}" {cc} rgba="0.85 0.2 0.2 1"/></body>'
           f'<body name="obj_cyl" pos="2.2 2 0.033"><freejoint name="obj_cyl_free"/>'
           f'<geom name="obj_cyl" type="cylinder" size="{O["cyl_r"]} {O["cyl_half"]}" mass="1" friction="{_f(C["obj_friction"])}" {cc} rgba="0.2 0.35 0.85 1"/></body>')
    if cam:
        world+=('<camera name="overview" pos="-0.9 -1.6 1.9" xyaxes="0.87 -0.49 0 0.24 0.43 0.87"/>'
                '<camera name="top" pos="0.6 0.0 2.3" xyaxes="0 -1 0 1 0 0"/>')
    xml=open(sc,encoding="utf-8").read().replace("</worldbody>",world+"</worldbody>",1)
    import re
    xml,n=re.subn(r'<option ([^>]*?)/>',lambda mo:f'<option {mo.group(1).replace(chr(116)+"imestep=\"0.002\"","")} {OPTION}><flag multiccd="enable"/></option>',xml,count=1); assert n==1
    return mujoco.MjModel.from_xml_string(xml,assets=a)

class Ids:
    """자주 쓰는 id 묶음"""
    def __init__(s,m):
        s.ee=m.site("left_ee").id; s.table=m.geom("table").id; s.goal=m.site("goal").id
        s.pads=[m.geom("left_pad_l").id,m.geom("left_pad_r").id]
        s.fq=[m.jnt_qposadr[m.joint(f"left_finger_{n}_joint").id] for n in "lr"]
        s.obj={k:m.body(f"obj_{k}").id for k in ("box","cyl")}
        s.objq={k:m.jnt_qposadr[m.joint(f"obj_{k}_free").id] for k in ("box","cyl")}
        s.objv={k:m.jnt_dofadr[m.joint(f"obj_{k}_free").id] for k in ("box","cyl")}
        s.objg={k:m.geom(f"obj_{k}").id for k in ("box","cyl")}
        s.act={m.actuator(i).name:i for i in range(m.nu)}
        robot=set()
        for g in range(m.ngeom):
            b=m.geom_bodyid[g]
            if b!=0 and m.body(b).name not in ("obj_box","obj_cyl") and m.geom_contype[g]: robot.add(g)
        s.robot=robot
        s.arm_geoms={g for g in robot if m.body(m.geom_bodyid[g]).name.startswith(("left_","right_"))}

def sample_task(seed):
    """seed → 물체 종류 · 위치 · yaw · 질량 (같은 seed = 같은 초기 상태, 재생 검증의 기준)"""
    r=np.random.default_rng(seed); T=TABLE
    kind=("box","cyl")[int(r.integers(2))]
    depth=r.uniform(*SPAWN["depth"]); y=r.uniform(*SPAWN["y"]); yaw=np.deg2rad(r.uniform(*SPAWN["yaw_deg"]))
    mass=float(np.exp(r.uniform(np.log(OBJ["mass"][0]),np.log(OBJ["mass"][1]))))   # 0.2 ~ 3 kg 로그 균등(가벼운 쪽도 고르게)
    return dict(seed=int(seed),kind=kind,pos=[T["edge_x"]+depth,y],yaw=float(yaw),mass=mass)

def obj_half_h(kind): return OBJ["box_half"] if kind=="box" else OBJ["cyl_half"]

def reset(m,d,task):
    """초기 상태: 베이스 원점 · 팔 영점(내림) · 그리퍼 벌림, 물체 2개 중 task 물체만 테이블 위(다른 하나는 바닥 멀리)"""
    I=Ids(m); mujoco.mj_resetData(m,d)
    for k in ("box","cyl"):
        m.body_mass[I.obj[k]]=task["mass"] if k==task["kind"] else 0.5
    # 질량을 바꿨으면 관성도 비례로 (균일 밀도 가정)
    for k in ("box","cyl"):
        g=I.objg[k]; sz=m.geom_size[g]; mm=m.body_mass[I.obj[k]]
        if k=="box": ii=np.full(3,mm*(2*sz[0])**2/6)
        else: ii=np.array([mm*(3*sz[0]**2+(2*sz[1])**2)/12]*2+[mm*sz[0]**2/2])
        m.body_inertia[I.obj[k]]=ii
    for k in ("box","cyl"):
        qa=I.objq[k]
        if k==task["kind"]:
            z=TABLE["top"]+obj_half_h(k)
            d.qpos[qa:qa+3]=[task["pos"][0],task["pos"][1],z]
            d.qpos[qa+3:qa+7]=[np.cos(task["yaw"]/2),0,0,np.sin(task["yaw"]/2)]
        else:
            d.qpos[qa:qa+3]=[-1.5,1.5 if k=="box" else -1.5,obj_half_h(k)]; d.qpos[qa+3:qa+7]=[1,0,0,0]
    d.qpos[I.fq]=GRIPPER["stroke"]
    d.ctrl[:]=0; d.ctrl[I.act["left_finger_l"]]=d.ctrl[I.act["left_finger_r"]]=GRIPPER["stroke"]
    mujoco.mj_forward(m,d); return I

def contacts(m,d,I):
    """현재 접촉 분류: 팔·몸 ↔ 테이블, 손가락 ↔ 물체, 손바닥 ↔ 물체 등"""
    out=dict(robot_table=[],pad_obj=set(),palm_obj=False,robot_obj_other=False,obj_table=False)
    objg=set(I.objg.values())
    for i in range(d.ncon):
        c=d.contact[i]; g1,g2=c.geom1,c.geom2
        for a,b in ((g1,g2),(g2,g1)):
            if a in I.robot and b==I.table: out["robot_table"].append(m.geom(a).name)
            if a in I.pads and b in objg: out["pad_obj"].add(a)
            if a in objg and b==I.table: out["obj_table"]=True
            if a in I.robot and a not in I.pads and b in objg:
                if m.geom(a).name.startswith("left_gripper"): out["palm_obj"]=True
                else: out["robot_obj_other"]=True
    return out

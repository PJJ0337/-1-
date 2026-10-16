# B1 공용 가정값 v0.1 정합 점검: 박종진 v0 URDF ↔ 함태훈 v0 URDF — 총질량 차이 분해 + 수평 중력토크 대조
# (1) 원본 두 URDF 그대로 (2) 함태훈 URDF에 공용 가정값(assumptions-v0.md v0.1 §2 C1~C4)을 적용한 사본
# 사용: python compare_b1_taehoon.py <B1 b1_humanoid_v0.urdf> <함태훈 our_robot_v0.urdf> <out.md>
import os,sys,re,json,numpy as np,mujoco,tempfile
B1,TH,OUT=sys.argv[1:4]
A=json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","..","b1","models","assumptions_b1.json"),encoding="utf-8"))
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
TOOL_B1=np.array([-0.009,0,-0.1989])   # left_tool0 in B1 left_elbow_link (build_v0_urdf.py TOOL)
def load(p): m=mujoco.MjModel.from_xml_path(p); d=mujoco.MjData(m); mujoco.mj_forward(m,d); return m,d
def groups(m):
    g=lambda n: float(m.body_mass[m.body(n).id]) if mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n)>=0 else 0.0
    r=dict(base=g("base_link"),torso=g("torso_link")+g("cad_frame"))
    for s in ("left","right"): r[s+"_arm"]=sum(g(f"{s}_{a}_link") for a in ARM)+g(f"{s}_tool0")
    r["total"]=float(sum(m.body_mass)); return r
def shoulder_z(m,d):
    j=m.joint("left_shoulder_pitch_joint").id; return float(d.xanchor[j][2])
def tool_in_cad(mb,db):
    c=mb.body("cad_frame").id; t=mb.body("left_tool0").id
    return db.xmat[c].reshape(3,3).T@(db.xpos[t]-db.xpos[c])
def torque(m,d,p_cad,payload):
    d.qpos[:]=0; d.qpos[m.jnt_qposadr[m.joint("left_shoulder_pitch_joint").id]]=-np.pi/2; mujoco.mj_forward(m,d)
    tau=float(d.qfrc_bias[m.jnt_dofadr[m.joint("left_shoulder_pitch_joint").id]])
    c=m.body("cad_frame").id; e=m.body("left_elbow_link").id
    # 페이로드 점: cad_frame 기준 영점 위치 → elbow 링크에 붙은 점으로 보고 수평 자세에서의 위치
    d0=mujoco.MjData(m); mujoco.mj_forward(m,d0)
    pw0=d0.xpos[c]+d0.xmat[c].reshape(3,3)@p_cad; pl=d0.xmat[e].reshape(3,3).T@(pw0-d0.xpos[e])
    pw=d.xpos[e]+d.xmat[e].reshape(3,3)@pl
    j=m.joint("left_shoulder_pitch_joint").id; r=pw-d.xanchor[j]; ax=d.xaxis[j]
    tau_pl=-float(np.cross(r,np.array([0,0,-9.81*payload]))@ax)   # qfrc_bias 부호(= 중력 상쇄 토크)
    return tau+tau_pl, pw
mb,db=load(B1); mh,dh=load(TH); pc=tool_in_cad(mb,db)
rows=[]
def rec(name,m,d):
    g=groups(m); t0,p=torque(m,d,pc,0); t3,_=torque(m,d,pc,3.0)
    rows.append(dict(model=name,**{k:round(v,3) for k,v in g.items()},shoulder_z=round(shoulder_z(m,d),4),tau0=round(t0,4),tau3=round(t3,4)))
rec("B1 v0 (박종진, 74.10)",mb,db); rec("함태훈 v0 원본 (f9c9eff)",mh,dh)
# (2) 함태훈 URDF에 공용 가정값 적용: 하체 30 + 상판 3.69, 몸통 box 5 → 8, 어깨 높이 1.20, 오른팔 = 왼팔 거울(공칭)
x=open(TH,encoding="utf-8").read()
b=A["base"]; t=A["torso"]
th_torso_cad=9.0634-5.0
x=re.sub(r'(<link name="base_link">\s*<inertial>.*?<mass value=")30(")',lambda mm: mm.group(1)+f'{b["mass_lower"]+b["mass_plate"]:.4g}'+mm.group(2),x,count=1,flags=re.S)
x=re.sub(r'(<link name="torso_link">\s*<inertial>.*?<mass value=")9.0634(")',lambda mm: mm.group(1)+f'{th_torso_cad+t["box_mass"]:.5g}'+mm.group(2),x,count=1,flags=re.S)
x=x.replace('<origin xyz="0 0 0.8"/></joint>',f'<origin xyz="0 0 {t["shoulder_z"]}"/></joint>',1)
x=x.replace('meshdir="meshes"',f'meshdir="{os.path.join(os.path.dirname(os.path.abspath(TH)),"meshes")}"')
tmp=os.path.join(tempfile.gettempdir(),"th_common.urdf"); open(tmp,"w",encoding="utf-8").write(x)
mc,dc=load(tmp)
# 오른팔 거울: 질량만 왼팔 값으로 (관성·형상은 함태훈 v1에서 거울로 내보냄 — 10/14 합의)
rec("함태훈 + 공용 가정값 v0.1",mc,dc)
r=rows[-1]; r["right_arm"]=rows[-1]["left_arm"]; r["total"]=round(r["total"]-groups(mc)["right_arm"]+groups(mc)["left_arm"],3); r["model"]+=" (+ 오른팔 거울 질량)"
L=["| 모델 | 베이스 kg | 몸통 kg | 왼팔 kg | 오른팔 kg | **총질량 kg** | 어깨 J1 높이 m | 수평 토크 0 kg N·m | 3 kg N·m |","|---|---|---|---|---|---|---|---|---|"]
for r in rows: L.append(f'| {r["model"]} | {r["base"]} | {r["torso"]} | {r["left_arm"]} | {r["right_arm"]} | **{r["total"]}** | {r["shoulder_z"]} | {abs(r["tau0"]):.2f} | {abs(r["tau3"]):.2f} |')
d0=rows[0]; d1=rows[1]
L+=["","총질량 차 분해 (B1 − 함태훈 원본):",f'- 베이스 {d0["base"]-d1["base"]:+.2f} kg (상판 3.69 kg을 B1만 하체 30 kg에 더함, 함태훈은 상판 메시만 있고 질량 없음)',f'- 몸통 {d0["torso"]-d1["torso"]:+.2f} kg (몸통 box B1 8 kg · 함태훈 5 kg)',f'- 팔 {d0["left_arm"]+d0["right_arm"]-d1["left_arm"]-d1["right_arm"]:+.2f} kg (B1 오른팔 = 왼팔 거울, 함태훈 오른팔 = CAD 오른팔)',f'- 합 {d0["total"]-d1["total"]:+.2f} kg',"",
   f'공용 가정값 적용 후 차 {rows[2]["total"]-d0["total"]:+.3f} kg (B1 더미 cad_frame · tool0 각 0.001 kg), 수평 토크 차 {abs(rows[2]["tau0"])-abs(d0["tau0"]):+.4f} N·m (0 kg) · {abs(rows[2]["tau3"])-abs(d0["tau3"]):+.4f} N·m (3 kg)']
open(OUT,"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

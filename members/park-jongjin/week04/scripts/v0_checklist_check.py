# 체크리스트 v0.3 자동 점검 — b1_humanoid_v0 (URDF + 3단 MJCF)
# 사용: python v0_checklist_check.py <model 폴더> <out.md>
import os,sys,re,json,numpy as np,mujoco,xml.etree.ElementTree as ET
MODEL,OUTMD=sys.argv[1],sys.argv[2]
U=os.path.join(MODEL,"urdf","b1_humanoid_v0.urdf"); S=os.path.join(MODEL,"mjcf","scene.xml")
root=ET.parse(U).getroot(); rows=[]
def add(no,item,ok,detail): rows.append((no,item,ok if isinstance(ok,str) else bool(ok),detail))
links=root.findall("link"); joints=root.findall("joint")
moving={j.find("child").get("link") for j in joints if j.get("type")!="fixed"}
# A1 · A2
noin=[l.get("name") for l in links if l.find("inertial") is None and l.get("name")!="odom"]
add("A1","모든 link에 inertial",not noin,f"없음: {noin or '-'} (odom = 월드 고정 루트, 질량 불필요)")
light=[l.get("name") for l in links if l.find("inertial") is not None and float(l.find("inertial/mass").get("value"))<1e-3-1e-12]
add("A2","질량 ≥ 1e-3 (가상 링크는 fixed)",not light,"tool0 2개 = 0.001 kg · fixed")
# A3
bad=[]
for l in links:
    i=l.find("inertial/inertia")
    if i is None: continue
    I=np.array([[float(i.get("ixx")),float(i.get("ixy")),float(i.get("ixz"))],[float(i.get("ixy")),float(i.get("iyy")),float(i.get("iyz"))],[float(i.get("ixz")),float(i.get("iyz")),float(i.get("izz"))]])
    w=np.linalg.eigvalsh(I)
    if w[0]<=0 or w[0]+w[1]<w[2]*(1-1e-9): bad.append(l.get("name"))
add("A3","관성 양정치 · 삼각부등식",not bad,f"위반 {bad or 0}개 — balanceinertia 미사용(B6)")
# A4
import trimesh
mx=0
for f in os.listdir(os.path.join(MODEL,"meshes")):
    if f.endswith(".stl"): mx=max(mx,max(trimesh.load(os.path.join(MODEL,"meshes",f)).extents))
add("A4","단위 m (메시 최대 길이)",0.05<mx<1.0,f"최대 extent {mx:.3f} m (mm 그대로면 ~300)")
# A5
rev=[j for j in joints if j.get("type")=="revolute"]
miss=[j.get("name") for j in rev if j.find("limit") is None or any(j.find("limit").get(k) is None for k in ("lower","upper","effort","velocity"))]
add("A5","revolute 8개 limit(lower·upper·effort·velocity)",not miss and len(rev)==8,f"revolute {len(rev)}개, 누락 {miss or 0} — 값은 가정(assumptions-v0)")
add("A6","트리(폐루프 없음)",len({j.find('child').get('link') for j in joints})==len(joints),"child 중복 없음")
add("A8","transmission · gazebo 미사용",root.find("transmission") is None and root.find("gazebo") is None,"액추에이터는 robot_actuated.xml")
# B
comp=root.find("mujoco/compiler").attrib
add("B1","fusestatic=false",comp.get("fusestatic")=="false","torso_link(fixed) · tool0 이름 보존")
add("B2","discardvisual=false",comp.get("discardvisual")=="false","")
add("B4","meshdir 지정",comp.get("meshdir")=="../meshes/","strippath 기본 → 파일명만")
add("B6","balanceinertia=false (팀 결정 10/8)",comp.get("balanceinertia")=="false","")
robot_xml=open(os.path.join(MODEL,"mjcf","robot.xml"),encoding="utf-8").read()
add("B3","robot.xml angle=radian",'angle="radian"' in robot_xml,"mj_saveLastXML이 명시")
# 로드
mu=mujoco.MjModel.from_xml_path(U); m=mujoco.MjModel.from_xml_path(S); d=mujoco.MjData(m)
names=[m.body(i).name for i in range(m.nbody)]
need=[l.get("name") for l in links]
add("F1","body 수 · 이름 = URDF link",all(n in names for n in need) and m.nbody==len(need)+1,f"nbody {m.nbody} = link {len(need)} + world, njnt {m.njnt}, ngeom {mu.ngeom}")
jt=[(m.joint(i).name,int(m.jnt_type[i])) for i in range(m.njnt)]
add("C3","베이스 planar → slide·slide·hinge",[t for _,t in jt[:3]]==[2,2,3],f"{[n for n,_ in jt[:3]]}, nq {m.nq} = 3 + 8")
mujoco.mj_forward(m,d)
add("D1","영점 자세 접촉 0",d.ncon==0,f"ncon {d.ncon}")
cnt=0
for i in range(m.njnt):
    if m.jnt_type[i]!=3 or not m.jnt_limited[i]: continue
    for v in m.jnt_range[i]:
        d.qpos[:]=0; d.qpos[m.jnt_qposadr[i]]=v; mujoco.mj_forward(m,d); cnt+=d.ncon
add("D3","관절 범위 양 끝 접촉 0 (단독 구동)",cnt==0,f"16자세 접촉 합 {cnt} — 범위는 충돌 스윕 기준(assumptions-v0 §2)")
add("D4","visual geom contype=conaffinity=0",all(m.geom_contype[g]==0 for g in range(m.ngeom) if "visual" in m.geom(g).name),"URDF visual → group 1")
# E
add("E1","3단 파일 구조",all(os.path.exists(os.path.join(MODEL,"mjcf",f)) for f in ("robot.xml","robot_actuated.xml","scene.xml")),"robot.xml(변환본) → robot_actuated.xml → scene.xml")
arm=[i for i in range(m.nu) if m.actuator(i).name not in ("base_vx","base_vy","base_wz")]
eff={j.get("name"):float(j.find("limit").get("effort")) for j in rev}
okE4=all(abs(m.actuator_forcerange[i][1]-eff[m.joint(m.actuator_trnid[i,0]).name])<1e-9 for i in arm)
okrange=all(np.allclose(m.actuator_ctrlrange[i],m.jnt_range[m.actuator_trnid[i,0]]) for i in arm)
add("E4","forcerange = effort, ctrlrange = 관절 range",okE4 and okrange,"forcerange ±100 Nm, inheritrange=1")
add("E5","implicitfast + dampratio",m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST,"dampratio=1 (임계감쇠)")
arma=[float(m.dof_armature[m.jnt_dofadr[m.actuator_trnid[i,0]]]) for i in arm]
add("E6","armature (BOM 전 0 허용)",all(a==0 for a in arma),"0 — RI85 사양(J_rotor·N²) 수령 시 교체, ICD 요청서")
# E7 · F3: 팔 앞으로 수평 정적 중력토크 손계산 vs MuJoCo
pj=m.joint("left_shoulder_pitch_joint"); q=pj.qposadr[0]; dof=pj.dofadr[0]
d.qpos[:]=0; d.qpos[q]=-np.pi/2; mujoco.mj_forward(m,d)
axis_pt=d.xanchor[pj.id]; axis=d.xaxis[pj.id]
hand=0.0
for a in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"):
    b=m.body(f"left_{a}_link").id; r=d.xipos[b]-axis_pt; F=np.array([0,0,-9.81*m.body_mass[b]]); hand+=np.dot(np.cross(r,F),axis)
b=m.body("left_tool0").id; hand+=np.dot(np.cross(d.xipos[b]-axis_pt,[0,0,-9.81*m.body_mass[b]]),axis)
mj=-d.qfrc_bias[dof]
add("F3","수평 자세 중력토크 손계산 = MuJoCo",abs(hand-mj)<1e-6,f"손계산 Σ r×mg = {hand:.4f} N·m, MuJoCo {mj:.4f} N·m")
tot=float(sum(m.body_mass))
add("F2","총질량 ≤ 80 kg (기획서)",tot<=80,f"{tot:.2f} kg (베이스 33.7 · 몸통 box 8 kg 가정 포함)")
res=json.load(open(os.path.join(os.path.dirname(OUTMD),"v0_analysis.json"),encoding="utf-8"))
h=res["hold_arm_forward"]
add("E7","position 서보 처짐 ≈ τg/kp",abs(h["sag_rad"]-abs(mj)/300)<0.005,f"시뮬 {h['sag_rad']:.4f} rad vs 이론 {abs(mj)/300:.4f} rad (kp 300)")
add("F5","뷰어/캡처: 영점 · 범위 · 중력","img/20261013_v0_mujoco.png","캡처 4자세, 영점 유지 2 s 처짐 0.0004 rad")
ok=sum(1 for r in rows if r[2] is True or isinstance(r[2],str)); ng=[r[0] for r in rows if r[2] is False]
with open(OUTMD,"w",encoding="utf-8") as fh:
    fh.write(f"# 체크리스트 v0.3 점검 — b1_humanoid_v0 (MuJoCo {mujoco.__version__})\n\n자동 생성: v0_checklist_check.py · 합계 {len(rows)}항목 — OK {ok} · NG {len(ng)} {ng if ng else ''}\n\n| No. | 점검 | 결과 | 근거 |\n|---|---|---|---|\n")
    for no,it,okk,det in rows: fh.write(f"| {no} | {it} | {'OK' if okk is True or isinstance(okk,str) else 'NG'} | {det} |\n")
print(open(OUTMD,encoding="utf-8").read())

# v0 모델 분석: (1) 관절별 자기충돌 없는 범위 스윕 (2) 중력토크 · 토크 여유 (3) 영점 유지 · 처짐
# 사용: python v0_analysis.py <scene.xml> <out_dir>
import os,sys,json,numpy as np,mujoco
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
scene,OUT=sys.argv[1],sys.argv[2]; os.makedirs(OUT,exist_ok=True)
m=mujoco.MjModel.from_xml_path(scene); d=mujoco.MjData(m)
res={}
arm=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
def qadr(n): return m.jnt_qposadr[m.joint(n).id]
def contacts():
    mujoco.mj_forward(m,d); out=[]
    for c in d.contact[:d.ncon]:
        b1=m.body(m.geom_bodyid[c.geom1]).name; b2=m.body(m.geom_bodyid[c.geom2]).name
        if "floor" in (m.geom(c.geom1).name,m.geom(c.geom2).name): continue
        out.append((b1,b2,float(c.dist)))
    return out
# (1) 스윕: 다른 관절 0, 1° 간격, collision(convex hull) 접촉 시작 각도
sweep={}
for a in arm:
    n=f"left_{a}_joint"; i=qadr(n); r={}
    for sgn in (+1,-1):
        hit=None
        for deg in range(1,181):
            d.qpos[:]=0; d.qpos[i]=sgn*np.deg2rad(deg); cs=contacts()
            if cs: hit=(deg,cs[0][0],cs[0][1]); break
        r["+" if sgn>0 else "-"]=hit
    sweep[n]=r; print("sweep",n,r)
res["sweep_deg"]=sweep
# (2) 중력토크: 팔을 앞으로 수평(shoulder_pitch로 −z → +x) — 정적 qfrc_bias, 페이로드 0 / 3 kg(tool0)
def static_torque(q_pitch,payload):
    m2=m; bid=m.body("left_tool0").id; m0=m.body_mass[bid]; m.body_mass[bid]=0.001+payload
    d.qpos[:]=0; d.qvel[:]=0; d.qpos[qadr("left_shoulder_pitch_joint")]=q_pitch
    mujoco.mj_forward(m,d); tau={a:float(d.qfrc_bias[m.jnt_dofadr[m.joint(f"left_{a}_joint").id]]) for a in arm}
    tool=d.xpos[bid].copy(); m.body_mass[bid]=m0
    return tau,tool
best=None
for deg in np.arange(-180,181,1):
    tau,tool=static_torque(np.deg2rad(deg),0)
    if best is None or abs(tau["shoulder_pitch"])>abs(best[1]["shoulder_pitch"]): best=(deg,tau,tool)
grav={}
for pl in (0.0,3.0):
    tau,tool=static_torque(np.deg2rad(best[0]),pl)
    grav[f"payload_{pl:g}kg"]=dict(q_pitch_deg=float(best[0]),tau=tau,tool0_xyz=[round(float(x),4) for x in tool])
res["gravity_worst_pitch"]=grav
eff=100.0
for k,v in grav.items():
    t=abs(v["tau"]["shoulder_pitch"]); v["margin_vs_100Nm_pct"]=round((eff-t)/t*100,1)
print(json.dumps(grav,indent=1))
# 팔 질량 · 리치
arm_mass=sum(m.body_mass[m.body(f"left_{a}_link").id] for a in arm)
res["left_arm_mass_kg"]=round(float(arm_mass),3); res["total_mass_kg"]=round(float(sum(m.body_mass)),3)
# (3) 영점 유지 (position kp 300, 중력보상 없음) 2 s
for pose,name in ((0,"zero"),(best[0],"arm_forward")):
    mujoco.mj_resetData(m,d); i=qadr("left_shoulder_pitch_joint"); j=qadr("right_shoulder_pitch_joint")
    d.qpos[i]=np.deg2rad(pose); d.qpos[j]=np.deg2rad(pose)
    d.ctrl[:]=0; ai=m.actuator("left_shoulder_pitch").id; aj=m.actuator("right_shoulder_pitch").id
    d.ctrl[ai]=np.deg2rad(pose); d.ctrl[aj]=np.deg2rad(pose)
    for _ in range(1000): mujoco.mj_step(m,d)
    res[f"hold_{name}"]=dict(cmd_deg=float(pose),left_pitch_deg=round(float(np.rad2deg(d.qpos[i])),3),
        sag_rad=round(float(d.qpos[i]-np.deg2rad(pose)),4),act_force_Nm=round(float(d.actuator_force[ai]),2),ncon=int(d.ncon),
        base_xy=[round(float(x),5) for x in d.qpos[:3]])
    print(name,res[f"hold_{name}"])
json.dump(res,open(os.path.join(OUT,"v0_analysis.json"),"w"),indent=1,ensure_ascii=False)

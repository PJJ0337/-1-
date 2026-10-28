# 함태훈 week06(2dcb548) 대조 (박종진 10/28): ① 뻗기 이상적 5차 궤적 역동역학(서보 없음) ② 고정 게인 영점 계단 응답
# 사용: 저장소 루트에서 cd b1/models/v1 && python ../../../members/park-jongjin/week06/scripts/check_taehoon_w06.py
import sys,json,numpy as np,mujoco,xml.etree.ElementTree as ET
sys.path.insert(0,'scripts'); from v1_mjio import load_model,_assets
SC='mjcf/scene.xml'
# (1) 역동역학: 정적 최악 pitch 자세로 T초 5차 궤적, 3 kg 끝점 질점
m=load_model(SC,strip_gravcomp=True); d=mujoco.MjData(m)
W=json.load(open('results/workspace_max_torque.json'))
qs=np.array(W['results']['3kg']['shoulder_pitch']['q'])
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
qa=[m.jnt_qposadr[m.joint(f"left_{a}_joint").id] for a in ARM]; da=[m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in ARM]
ee=m.site("left_ee").id; G=np.array([0,0,-9.81])
for T in (2.0,1.5,1.0,0.6):
    dt=0.001; t=np.arange(0,T+dt,dt); u=t/T
    s=10*u**3-15*u**4+6*u**5; sd=(30*u**2-60*u**3+30*u**4)/T; sdd=(60*u-180*u**2+120*u**3)/T**2
    taus=[]; P=[]
    for k in range(len(t)):
        d.qpos[:]=0; d.qvel[:]=0; d.qacc[:]=0
        d.qpos[qa]=s[k]*qs; d.qvel[da]=sd[k]*qs; d.qacc[da]=sdd[k]*qs
        mujoco.mj_inverse(m,d); jac=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jac,None,ee)
        P.append(d.site_xpos[ee].copy()); taus.append((d.qfrc_inverse[da].copy(),jac[:,da].copy()))
    P=np.array(P); A=np.gradient(np.gradient(P,dt,axis=0),dt,axis=0)
    tp=[abs(tau[0]+(J.T@(3.0*(A[k]-G)))[0]) for k,(tau,J) in enumerate(taus)]
    print(f"T {T}: pitch |tau| max {max(tp):.2f}  peak qdot {np.abs(sd).max()*np.abs(qs).max():.2f}")
# (2) 고정 게인 계단 응답 (a2_doc, gravcomp on)
G6={"shoulder_pitch":(400,31.0),"shoulder_roll":(350,27.5),"shoulder_yaw":(300,15.0),"elbow":(300,11.0)}
a=_assets(SC); r=ET.fromstring(a["b1.xml"].decode())
for act in r.iter("position"):
    n=act.get("name") or ""
    for j,(kp,kv) in G6.items():
        if n.endswith(j): act.set("kp",str(kp)); act.set("kv",str(kv)); pass
for dd in r.iter("default"):
    p=dd.find("position")
    if p is not None and "dampratio" in p.attrib: del p.attrib["dampratio"]
a["b1.xml"]=ET.tostring(r,encoding="unicode").encode()
m2=mujoco.MjModel.from_xml_string(open(SC).read(),assets=a); d2=mujoco.MjData(m2)
for j in ARM:
    ai=m2.actuator(f"left_{j}").id; print(j,"kp",m2.actuator_gainprm[ai][0],"kv",-m2.actuator_biasprm[ai][2])
    mujoco.mj_resetData(m2,d2); d2.ctrl[ai]=0.2; y=[]
    for k in range(1500): mujoco.mj_step(m2,d2); y.append(d2.qpos[m2.jnt_qposadr[m2.joint(f"left_{j}_joint").id]])
    y=np.array(y); tt=np.arange(1,1501)*0.002; b=np.abs(y-0.2)>0.004
    print(f"   OS {max(0,(y.max()-0.2)/0.2*100):.2f} %  settle {tt[np.where(b)[0][-1]] if b.any() else 0:.3f} s")

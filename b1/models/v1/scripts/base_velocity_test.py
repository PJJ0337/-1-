# 홀로노믹 베이스(C3 평면 3-DOF) 속도 명령 시험 — 명령 vs 실제 qvel (week05 화 실습)
# 시나리오: 0~2 s vx 0.5 → 2~4 s vy 0.5 → 4~6 s wz 0.8 rad/s → 6~8 s yaw 돈 상태에서 vx 0.5 (body 기준 명령 → world 변환, 체크리스트 C3-2)
# 사용: MUJOCO_GL=osmesa python base_velocity_test.py   → ../results/base_velocity_test.{csv,json}, ../img/base_velocity_test.png
import os,sys,json,numpy as np,mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if "CJK" in f or "Nanum" in f or "malgun" in f.lower(): plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
plt.rcParams["axes.unicode_minus"]=False
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE); from v1_mjio import load_model
m=load_model(os.path.join(V1,"mjcf","scene.xml")); d=mujoco.MjData(m)
J=["base_planar_joint_TX","base_planar_joint_TY","base_planar_joint_RZ"]; dof=[m.jnt_dofadr[m.joint(j).id] for j in J]; qa=[m.jnt_qposadr[m.joint(j).id] for j in J]
A=[m.actuator(n).id for n in ("base_vx","base_vy","base_wz")]
def cmd_body(t):
    if t<2: return (0.5,0,0)
    if t<4: return (0,0.5,0)
    if t<6: return (0,0,0.8)
    if t<8: return (0.5,0,0)
    return (0,0,0)
T=9.0; log=[]
for k in range(int(T/m.opt.timestep)):
    t=d.time; vb=cmd_body(t); yaw=d.qpos[qa[2]]
    vw=(np.cos(yaw)*vb[0]-np.sin(yaw)*vb[1], np.sin(yaw)*vb[0]+np.cos(yaw)*vb[1])   # body → world (slide 축 = world 고정, C3-2)
    d.ctrl[A[0]],d.ctrl[A[1]],d.ctrl[A[2]]=vw[0],vw[1],vb[2]
    # 팔은 영점 유지
    mujoco.mj_step(m,d)
    log.append([t,vw[0],vw[1],vb[2],*[d.qvel[i] for i in dof],*[d.qpos[i] for i in qa]])
L=np.array(log); np.savetxt(os.path.join(V1,"results","base_velocity_test.csv"),L,delimiter=",",header="t,cmd_vx_w,cmd_vy_w,cmd_wz,qvel_x,qvel_y,qvel_yaw,x,y,yaw",fmt="%.5f")
# 지표: 각 구간 정상상태 오차(구간 끝 0.5 s 평균), 63 % 도달 시간
res={}
for name,(t0,t1,ci,vi) in {"vx":(0,2,1,4),"vy":(2,4,2,5),"wz":(4,6,3,6),"vx_after_yaw":(6,8,1,4)}.items():
    seg=L[(L[:,0]>=t0)&(L[:,0]<t1)]; ss=seg[seg[:,0]>=t1-0.5]
    err=float(np.abs(ss[:,vi]-ss[:,ci]).mean()); target=seg[-1,ci]
    tau=next((float(r[0]-t0) for r in seg if abs(r[vi])>=0.632*abs(target)),None)
    res[name]=dict(cmd=round(float(target),4),ss_err=round(err,6),t63_s=round(tau,4) if tau is not None else None)
# body 기준 속도 확인(6~8 s): world qvel을 yaw로 되돌림
seg=L[(L[:,0]>=7.5)&(L[:,0]<8)]; vbx=np.cos(seg[:,9])*seg[:,4]+np.sin(seg[:,9])*seg[:,5]; vby=-np.sin(seg[:,9])*seg[:,4]+np.cos(seg[:,9])*seg[:,5]
res["vx_after_yaw"]["body_v"]=[round(float(vbx.mean()),4),round(float(vby.mean()),4)]
res["yaw_after_wz_deg"]=round(float(np.rad2deg(L[L[:,0]<6][-1,9])),2); res["final_xy"]=[round(float(L[-1,7]),4),round(float(L[-1,8]),4)]
res["kv"]=dict(vx=float(m.actuator_gainprm[A[0]][0]),wz=float(m.actuator_gainprm[A[2]][0]))
json.dump(res,open(os.path.join(V1,"results","base_velocity_test.json"),"w"),indent=1,ensure_ascii=False); print(json.dumps(res,ensure_ascii=False))
fig,ax=plt.subplots(1,3,figsize=(15,3.8))
for a,(ci,vi,lab,u) in zip(ax,((1,4,"x (world)","m/s"),(2,5,"y (world)","m/s"),(3,6,"yaw","rad/s"))):
    a.plot(L[:,0],L[:,ci],"--",color="#888",label="명령"); a.plot(L[:,0],L[:,vi],color="#1f6fb4",label="실제 qvel")
    a.set_title(f"베이스 {lab}"); a.set_xlabel("t [s]"); a.set_ylabel(u); a.grid(alpha=.3); a.legend(fontsize=8)
fig.suptitle("v1 홀로노믹 베이스 속도 명령 vs qvel (velocity 액추에이터, 6~8 s = yaw 회전 후 body 기준 vx 0.5 → world 변환)",fontsize=11)
plt.tight_layout(); os.makedirs(os.path.join(V1,"img"),exist_ok=True); plt.savefig(os.path.join(V1,"img","base_velocity_test.png"),dpi=110)

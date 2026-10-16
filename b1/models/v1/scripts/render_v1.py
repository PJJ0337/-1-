# v1 캡처 + 액추에이터 동작 점검 + 중력보상 처짐 그래프 (오프스크린). 사용: MUJOCO_GL=osmesa python render_v1.py
#  - img/v1_poses.png      4자세 (site ee · IMU 표시)
#  - img/v1_collision.png  collision(볼록 껍질 · box)
#  - img/v1_gravcomp.png   팔 앞으로 수평 유지: 중력보상 없음 vs 있음 (pitch 오차 · 액추에이터 토크)
#  - results/actuator_check.json  액추에이터 11개 각각 명령 → 관절이 명령 방향 · 크기로 움직이는지 (뷰어 슬라이더 점검의 자동판)
import os,sys,json,numpy as np,mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if "CJK" in f or "Nanum" in f: plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
plt.rcParams["axes.unicode_minus"]=False
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE); from v1_mjio import load_model
m=load_model(os.path.join(V1,"mjcf","scene.xml")); d=mujoco.MjData(m); IMG=os.path.join(V1,"img"); os.makedirs(IMG,exist_ok=True)
qa=lambda n: m.jnt_qposadr[m.joint(n).id]; da=lambda n: m.jnt_dofadr[m.joint(n).id]
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; SIDES=("left","right")
r=mujoco.Renderer(m,720,800)
def shot(pose,groups=(1,2,3),az=145,el=-12,dist=2.1,look=(0,0,0.85)):
    mujoco.mj_resetData(m,d)
    for j,v in pose.items(): d.qpos[qa(j)]=v
    mujoco.mj_forward(m,d)
    cam=mujoco.MjvCamera(); cam.lookat[:]=look; cam.distance=dist; cam.azimuth=az; cam.elevation=el
    opt=mujoco.MjvOption(); opt.geomgroup[:]=0; opt.sitegroup[:]=0
    for g in groups: opt.geomgroup[g]=1
    opt.sitegroup[3]=1
    r.update_scene(d,cam,opt); return r.render(),d.ncon
P={"영점 (팔 내림 · 팔꿈치 전방 굽힘 방향)":{},
   "shoulder_pitch −90° (양팔 앞으로 수평)":{f"{s}_shoulder_pitch_joint":-np.pi/2 for s in SIDES},
   "shoulder_roll +1.2 rad (외전, 같은 명령 → 좌우 대칭)":{f"{s}_shoulder_roll_joint":1.2 for s in SIDES},
   "pitch −0.6 · roll 0.3 · yaw 0.5 · elbow +1.4 (양팔 같은 각)":{f"{s}_{a}_joint":v for s in SIDES for a,v in zip(ARM,(-0.6,0.3,0.5,1.4))}}
fig,ax=plt.subplots(1,4,figsize=(19,6.4))
for a,(t,p) in zip(ax,P.items()):
    img,nc=shot(p); a.imshow(img); a.set_title(f"{t}\n바닥 외 접촉 {sum(1 for c in d.contact[:d.ncon] if 'floor' not in (m.geom(c.geom1).name,m.geom(c.geom2).name))}개",fontsize=10.5); a.axis("off")
fig.suptitle("b1_v1 — 양팔(오른팔 = 왼팔 미러 생성) + 홀로노믹 베이스 3-DOF + 상판 별도 링크 + 그리퍼 임시 box · ee site(주황) · IMU site(파랑)   [build_v1.py → mjcf/scene.xml, MuJoCo 3.15.0]",fontsize=11.5)
plt.tight_layout(); plt.savefig(os.path.join(IMG,"v1_poses.png"),dpi=105); plt.close()
img,_=shot({},groups=(0,2,3)); plt.imsave(os.path.join(IMG,"v1_collision.png"),img)
# 액추에이터 점검: 각 액추에이터에 범위의 40 % 명령, 1.5 s 후 응답
chk=[]
for i in range(m.nu):
    mujoco.mj_resetData(m,d); nm=m.actuator(i).name; j=m.actuator_trnid[i][0]; jn=m.joint(j).name
    lo,hi=m.actuator_ctrlrange[i]; u=0.4*hi if hi>0 else 0.4*lo
    d.ctrl[i]=u; base=np.array(d.ctrl)
    for k in range(750):
        # 팔은 중력보상해 다른 관절 처짐 영향 제거
        for s in SIDES:
            for a in ARM:
                ai=m.actuator(f"{s}_{a}").id
                if ai!=i: d.ctrl[ai]=d.qfrc_bias[da(f"{s}_{a}_joint")]/m.actuator_gainprm[ai][0]
                else: d.ctrl[ai]=u+d.qfrc_bias[da(jn)]/m.actuator_gainprm[ai][0]
        mujoco.mj_step(m,d)
    typ="velocity" if nm.startswith("base") else "position"
    got=float(d.qvel[m.jnt_dofadr[j]]) if typ=="velocity" else float(d.qpos[m.jnt_qposadr[j]])
    chk.append(dict(actuator=nm,joint=jn,type=typ,cmd=round(float(u),4),response=round(got,4),err=round(abs(got-u),4),
                    ok=bool(np.sign(got)==np.sign(u) and abs(got-u)<=0.02*abs(u)+1e-3),ncon=int(sum(1 for c in d.contact[:d.ncon] if 'floor' not in (m.geom(c.geom1).name,m.geom(c.geom2).name)))))
json.dump(chk,open(os.path.join(V1,"results","actuator_check.json"),"w"),indent=1,ensure_ascii=False)
print("actuator ok",sum(c["ok"] for c in chk),"/",len(chk))
# 중력보상 처짐: left 팔 앞으로 수평 유지 3 s
out={}
for gc in (False,True):
    mujoco.mj_resetData(m,d); q0=-np.pi/2
    for s in SIDES: d.qpos[qa(f"{s}_shoulder_pitch_joint")]=q0
    mujoco.mj_forward(m,d); T=[];E=[];F=[]
    for k in range(1500):
        for s in SIDES:
            for a in ARM:
                j=f"{s}_{a}_joint"; ai=m.actuator(f"{s}_{a}").id; tgt=q0 if a=="shoulder_pitch" else 0.0
                d.ctrl[ai]=tgt+(d.qfrc_bias[da(j)]/m.actuator_gainprm[ai][0] if gc else 0)
        mujoco.mj_step(m,d); T.append(d.time); E.append(np.rad2deg(d.qpos[qa("left_shoulder_pitch_joint")]-q0)); F.append(d.actuator_force[m.actuator("left_shoulder_pitch").id])
    out[gc]=(np.array(T),np.array(E),np.array(F))
kp=m.actuator_gainprm[m.actuator("left_shoulder_pitch").id][0]
mujoco.mj_resetData(m,d); d.qpos[qa("left_shoulder_pitch_joint")]=-np.pi/2; mujoco.mj_forward(m,d); tg=d.qfrc_bias[da("left_shoulder_pitch_joint")]
fig,ax=plt.subplots(1,2,figsize=(12,3.6))
for gc,c,lab in ((False,"#c0392b","중력보상 없음"),(True,"#1f6fb4","중력보상 (목표 + τg/kp)")):
    T,E,F=out[gc]; ax[0].plot(T,E,color=c,label=f"{lab}: 최종 {E[-1]:+.3f}°"); ax[1].plot(T,F,color=c,label=lab)
ax[0].axhline(np.rad2deg(-tg/kp),ls=":",color="#888",label=f"이론 처짐 −bias/kp = {np.rad2deg(-tg/kp):+.2f}°")
ax[0].set_title("left shoulder_pitch 오차 (목표 −90°, 팔 앞으로 수평)"); ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("°"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
ax[1].set_title(f"액추에이터 토크 (정적 중력 토크 {abs(tg):.2f} N·m)"); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("N·m"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(IMG,"v1_gravcomp.png"),dpi=110)
g=dict(tau_g=round(float(tg),3),kp=float(kp),theory_sag_deg=round(float(np.rad2deg(-tg/kp)),3),final_sag_nogc_deg=round(float(out[False][1][-1]),4),final_sag_gc_deg=round(float(out[True][1][-1]),5),
       force_gc=round(float(out[True][2][-1]),3))
json.dump(g,open(os.path.join(V1,"results","gravcomp.json"),"w"),indent=1); print(g)

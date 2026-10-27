# 작업공간 분석 (week06 커리큘럼 월: 순기구학 · 작업공간) — 박종진
#   1) 팔별 관절 범위 안 균일 무작위 샘플 N개(기본 30,000) → mj_forward → ee site 위치 (자기 충돌 자세 제외 후 ≥ 20,000)
#   2) 양팔 공유 작업공간 = 두 팔 점구름이 모두 들어간 복셀(기본 3 cm)
#   3) 베이스 상판 기준 높이 · 전방 도달거리, 높이 구간별 최대 전방 도달 (A팀 피드백 표)
#   4) 베이스 이동 포함 시 확장 (평면 3-DOF → 수평 무제한, 높이 대역은 그대로)
#   5) 토크 예산표용: 도달 가능(충돌 없는) 자세 중 관절별 정적 중력 토크 최대 자세 → results/workspace_max_torque.json (페이로드 0 / 0.5 / 1.0 / 3 kg)
# 결과: results/workspace.{json,md}, results/workspace_max_torque.json, results/workspace_points.npz, img/workspace.png
# 사용: python workspace.py [N] [voxel_m]
import os,sys,json,time,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); MODELS=os.path.dirname(V1)
sys.path.insert(0,HERE); from v1_mjio import load_model
A=json.load(open(os.path.join(MODELS,"assumptions_b1.json"),encoding="utf-8"))
N=int(sys.argv[1]) if len(sys.argv)>1 else 30000; VOX=float(sys.argv[2]) if len(sys.argv)>2 else 0.03
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; SIDES=("left","right")
PAYLOADS=(0.0,0.5,1.0,3.0); G=np.array([0,0,-9.81])
PLATE_Z=A["base"]["plate_top_z"]; PX,PY=A["base"]["plate_xy"]
RATED_CRIT=float(A["pdr_torque"]["rated_criterion_Nm"])
A2={"RI85":dict(rated=72.6,peak=181.5),"TQ":dict(rated=118.8,peak=381.7)}   # A2 사양서 10/19 (요청서 v0.3 §5)

m=load_model(os.path.join(V1,"mjcf","scene.xml")); d=mujoco.MjData(m)
qa=lambda n: m.jnt_qposadr[m.joint(n).id]; da=lambda n: m.jnt_dofadr[m.joint(n).id]
FLOOR=m.geom("floor").id
def self_contacts():
    return sum(1 for c in d.contact[:d.ncon] if FLOOR not in (c.geom1,c.geom2))

def sample_arm(side,rng):
    lims=np.array([m.jnt_range[m.joint(f"{side}_{a}_joint").id] for a in ARM])
    Q=rng.uniform(lims[:,0],lims[:,1],size=(N,4))
    ee=m.site(f"{side}_ee").id; P=np.zeros((N,3)); ok=np.zeros(N,bool); T0=np.zeros((N,4)); JT=np.zeros((N,4,3))
    jac=np.zeros((3,m.nv)); dofs=[da(f"{side}_{a}_joint") for a in ARM]; adr=[qa(f"{side}_{a}_joint") for a in ARM]
    for i,q in enumerate(Q):
        d.qpos[:]=0; d.qvel[:]=0; d.qpos[adr]=q
        mujoco.mj_forward(m,d)
        P[i]=d.site_xpos[ee]; ok[i]=self_contacts()==0
        T0[i]=d.qfrc_bias[dofs]                                   # qvel = 0 → 중력 토크 (팔 + 그리퍼 box)
        mujoco.mj_jacSite(m,d,jac,None,ee); JT[i]=jac[:,dofs].T   # 페이로드 F = m·g at ee → τ_payload = −Jᵀ F (qfrc_bias 부호 규약과 같게)
    return Q,P,ok,T0,JT

t0=time.time(); rng=np.random.default_rng(6)
S={s:sample_arm(s,rng) for s in SIDES}
R={"N_per_arm":N,"voxel_m":VOX,"plate_top_z":PLATE_Z,"frame":"base_link = world (베이스 영점), z = 바닥 기준","arms":{}}
for s in SIDES:
    Q,P,ok,_,_=S[s]; Pv=P[ok]
    d.qpos[:]=0; mujoco.mj_forward(m,d); sh=d.xpos[m.body(f"{s}_shoulder_pitch_link").id].copy()
    r=np.linalg.norm(Pv-sh,axis=1)
    hb=[]
    for z0 in np.arange(0.6,1.81,0.1):
        sel=(Pv[:,2]>=z0)&(Pv[:,2]<z0+0.1)
        hb.append(dict(z_band=[round(z0,2),round(z0+0.1,2)],above_plate=[round(z0-PLATE_Z,2),round(z0+0.1-PLATE_Z,2)],n=int(sel.sum()),
                       x_max=round(float(Pv[sel,0].max()),3) if sel.any() else None,
                       beyond_plate_edge=round(float(Pv[sel,0].max()-PX/2),3) if sel.any() else None))
    R["arms"][s]=dict(n_valid=int(ok.sum()),n_self_collision=int((~ok).sum()),shoulder=[round(float(x),4) for x in sh],
        x=[round(float(Pv[:,0].min()),3),round(float(Pv[:,0].max()),3)],y=[round(float(Pv[:,1].min()),3),round(float(Pv[:,1].max()),3)],
        z=[round(float(Pv[:,2].min()),3),round(float(Pv[:,2].max()),3)],
        above_plate=[round(float(Pv[:,2].min()-PLATE_Z),3),round(float(Pv[:,2].max()-PLATE_Z),3)],
        reach_from_shoulder_max=round(float(r.max()),3),reach_from_shoulder_min=round(float(r.min()),3),
        forward_reach_max=round(float(Pv[:,0].max()),3),forward_reach_z=round(float(Pv[np.argmax(Pv[:,0]),2]),3),
        forward_beyond_plate_edge=round(float(Pv[:,0].max()-PX/2),3),
        cross_midline_max=round(float(-Pv[:,1].min() if s=="left" else Pv[:,1].max()),3),
        height_bands=hb)

# 공유 작업공간: 복셀 점유 교집합. 공유 후보 = 정중면 근처 얇은 판(|y| ≤ 0.15 m)이라 균일 샘플로는 복셀당 점이 2개 안팎 →
#   추가 샘플 NS개(팔당, mj_kinematics + mj_collision만)로 판 안 점을 조밀하게 채운 뒤 교집합 (부피 과소 추정 방지)
NS=int(os.environ.get("WS_SHARED_N","300000"))
def dense_slab(side,rng):
    lims=np.array([m.jnt_range[m.joint(f"{side}_{a}_joint").id] for a in ARM]); adr=[qa(f"{side}_{a}_joint") for a in ARM]; ee=m.site(f"{side}_ee").id
    Q=rng.uniform(lims[:,0],lims[:,1],size=(NS,4)); out=[]
    for q in Q:
        d.qpos[:]=0; d.qpos[adr]=q; mujoco.mj_kinematics(m,d)
        p=d.site_xpos[ee]
        if abs(p[1])>0.15: continue
        mujoco.mj_comPos(m,d); mujoco.mj_collision(m,d)
        if self_contacts()==0: out.append(p.copy())
    return np.array(out)
def vox(P): return set(map(tuple,np.floor(P/VOX).astype(int)))
DL=dense_slab("left",rng); DR=dense_slab("right",rng)
VL=vox(np.vstack([S["left"][1][S["left"][2]],DL])); VR=vox(np.vstack([S["right"][1][S["right"][2]],DR])); VS=VL&VR
C=(np.array(sorted(VS))+0.5)*VOX if VS else np.zeros((0,3))
R["shared"]=dict(dense_slab_samples_per_arm=NS,dense_slab_points=[len(DL),len(DR)],n_voxel_left=len(VL),n_voxel_right=len(VR),n_voxel_shared=len(VS),
    vol_left_m3=round(len(VL)*VOX**3,4),vol_right_m3=round(len(VR)*VOX**3,4),vol_shared_m3=round(len(VS)*VOX**3,4),
    shared_pct_of_left=round(len(VS)/len(VL)*100,1),
    x=[round(float(C[:,0].min()),3),round(float(C[:,0].max()),3)] if len(C) else None,
    y=[round(float(C[:,1].min()),3),round(float(C[:,1].max()),3)] if len(C) else None,
    z=[round(float(C[:,2].min()),3),round(float(C[:,2].max()),3)] if len(C) else None,
    above_plate=[round(float(C[:,2].min()-PLATE_Z),3),round(float(C[:,2].max()-PLATE_Z),3)] if len(C) else None,
    centroid=[round(float(x),3) for x in C.mean(0)] if len(C) else None,
    note="복셀 점유 기반(|y| ≤ 0.15 m 판은 추가 샘플로 조밀화, 그래도 경계 복셀 일부 누락 가능 → 부피는 하한). 양손 협업 영역 = 몸통 정면 y ≈ 0 부근")
# 공유 영역 중 상판 위 정면(x>0) 대표점: 양손 동시 집기 목표 후보
if len(C):
    fr=C[C[:,0]>0.15]; R["shared"]["front_candidates"]=[[round(float(v),3) for v in p] for p in fr[np.argsort(fr[:,2])][:: max(1,len(fr)//5)][:5]]

# 베이스 이동 포함 확장 (평면 3-DOF: x · y 무제한, yaw 360°)
zl=R["arms"]["left"]["z"]
R["with_base"]=dict(z_band=zl,above_plate=R["arms"]["left"]["above_plate"],
    horizontal="무제한 (base_x · base_y slide, 실제는 주행 공간 · 장애물로 제한)",
    yaw="base_yaw 360° → 한 팔로 전 방위 접근, 공유 작업공간도 원하는 방위로 회전",
    note="베이스는 높이를 바꾸지 못함 → 도달 높이 대역은 팔 단독과 같음. 상판 높이(0.40 m) ~ 그 위 0.20 m 와 0.605 m 이하(예: 바닥 · 낮은 선반)는 베이스 이동으로도 도달 불가")

# 토크 예산표용: 관절별 정적 토크 최대 자세 (도달 가능 = 충돌 없는 샘플) + 국소 최적화
from scipy.optimize import minimize
def tau_static(side,q,pl):
    d.qpos[:]=0; d.qvel[:]=0
    for a,v in zip(ARM,q): d.qpos[qa(f"{side}_{a}_joint")]=v
    mujoco.mj_forward(m,d); dofs=[da(f"{side}_{a}_joint") for a in ARM]
    jac=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jac,None,m.site(f"{side}_ee").id)
    return d.qfrc_bias[dofs]-pl*(jac[:,dofs].T@G), d.site_xpos[m.site(f"{side}_ee").id].copy(), self_contacts()
WT={"_note":"정적 중력 토크 |τ| 최대 자세 (qvel = 0, qfrc_bias + 페이로드 −Jᵀ·m·g at {side}_ee). 후보 = 작업공간 충돌 없는 샘플 → 상위 6자세 L-BFGS-B(관절 범위) → 최적점 자기 충돌 0 확인. 왼팔 기준(오른팔 = 거울 대칭, 크기 동일 · 부호 규칙은 mirror_rules.md)",
    "joint_order":ARM,"units":{"q":"rad","q_deg":"deg","ee_pos":"m (base_link)","tau":"N·m (4관절, 부호 = qfrc_bias 규약: 유지에 필요한 구동 토크와 같은 부호)"},
    "criterion":{"pdr_rated_Nm":RATED_CRIT,"A2":A2,"rule":"정격 ≥ |τ| × 1.2 (팀장 10/19)"},"results":{}}
Q,P,ok,T0,JT=S["left"]; LB=[m.jnt_range[m.joint(f"left_{a}_joint").id][0] for a in ARM]; UB=[m.jnt_range[m.joint(f"left_{a}_joint").id][1] for a in ARM]
for pl in PAYLOADS:
    tau_all=T0-pl*(JT@G); out={}
    for i,a in enumerate(ARM):
        idx=np.where(ok)[0][np.argsort(-np.abs(tau_all[ok,i]))[:6]]
        best=(abs(tau_all[idx[0],i]),Q[idx[0]]); samp=best[0]
        for k in idx:
            r=minimize(lambda x:-abs(tau_static("left",x,pl)[0][i]),Q[k],method="L-BFGS-B",bounds=list(zip(LB,UB)))
            if -r.fun>best[0] and tau_static("left",r.x,pl)[2]==0: best=(float(-r.fun),r.x)
        tau,pos,nc=tau_static("left",best[1],pl)
        act="TQ" if a!="elbow" else "RI85"
        out[a]=dict(q=[round(float(x),4) for x in best[1]],q_deg=[round(float(np.rad2deg(x)),1) for x in best[1]],ee_pos=[round(float(x),4) for x in pos],
                    tau=[round(float(x),3) for x in tau],tau_abs_max=round(float(abs(tau[i])),3),sample_max=round(float(samp),3),self_contacts=nc,
                    margin_vs_pdr52_pct=round((RATED_CRIT-abs(tau[i]))/abs(tau[i])*100,1),
                    margin_vs_RI85_rated_pct=round((A2["RI85"]["rated"]-abs(tau[i]))/abs(tau[i])*100,1),
                    margin_vs_A2_rated_pct=round((A2[act]["rated"]-abs(tau[i]))/abs(tau[i])*100,1),A2_actuator=act)
    WT["results"][f"{pl:g}kg"]=out
R["elapsed_s"]=round(time.time()-t0,1)
os.makedirs(os.path.join(V1,"results"),exist_ok=True)
json.dump(R,open(os.path.join(V1,"results","workspace.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
json.dump(WT,open(os.path.join(V1,"results","workspace_max_torque.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
np.savez_compressed(os.path.join(V1,"results","workspace_points.npz"),**{f"{s}_q":S[s][0][S[s][2]].astype(np.float32) for s in SIDES},**{f"{s}_ee":S[s][1][S[s][2]].astype(np.float32) for s in SIDES},shared_voxel_centers=C.astype(np.float32))

# ---- 그림: 위 · 옆 · 앞 3면 (왼팔 slot1 파랑, 오른팔 slot2 주황, 공유 복셀 slot7 보라)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams.update({"font.size":9,"axes.edgecolor":"#8a8986","axes.linewidth":0.6,"xtick.color":"#52514e","ytick.color":"#52514e","axes.labelcolor":"#52514e"})
for f in ("NanumGothic","Noto Sans CJK KR","Malgun Gothic"):
    if any(f in x.name for x in matplotlib.font_manager.fontManager.ttflist): plt.rcParams["font.family"]=f; break
CL={"left":"#2a78d6","right":"#eb6834"}; CS="#6250d6"
fig,ax=plt.subplots(1,3,figsize=(13,4.6),constrained_layout=True)
views=[(0,1,"위 (x–y)","x 전방 [m]","y 좌측 [m]"),(0,2,"옆 (x–z)","x 전방 [m]","z [m]"),(1,2,"앞 (y–z)","y 좌측 [m]","z [m]")]
for k,(i,j,tt,xl,yl) in enumerate(views):
    a=ax[k]
    for s in SIDES:
        Pv=S[s][1][S[s][2]]; a.scatter(Pv[:,i],Pv[:,j],s=0.6,c=CL[s],alpha=0.18,linewidths=0,rasterized=True)
    if len(C): a.scatter(C[:,i],C[:,j],s=3,c=CS,alpha=0.5,linewidths=0)
    if (i,j)==(0,1): a.add_patch(plt.Rectangle((-PX/2,-PY/2),PX,PY,fill=False,ec="#52514e",lw=1,ls="--"))
    else:
        a.axhline(PLATE_Z,color="#52514e",lw=1,ls="--"); a.text(-0.85 if i==1 else -0.6,PLATE_Z+0.025,"상판 윗면 0.40 m",color="#52514e",fontsize=8)
        a.axhline(R["arms"]["left"]["z"][0],color="#8a8986",lw=0.6,ls=":"); a.text(-0.85 if i==1 else -0.6,R["arms"]["left"]["z"][0]-0.06,f'최저 도달 {R["arms"]["left"]["z"][0]:.3f} m',color="#52514e",fontsize=8)
    a.set_title(tt,color="#0b0b0b",fontsize=10,loc="left"); a.set_xlabel(xl); a.set_ylabel(yl); a.set_aspect("equal"); a.grid(color="#e4e3df",lw=0.5)
from matplotlib.lines import Line2D
h=[Line2D([],[],marker="o",ls="",color=CL["left"],label=f'왼팔 ({R["arms"]["left"]["n_valid"]:,}점)'),Line2D([],[],marker="o",ls="",color=CL["right"],label=f'오른팔 ({R["arms"]["right"]["n_valid"]:,}점)'),
   Line2D([],[],marker="s",ls="",color=CS,label=f'공유 {R["shared"]["vol_shared_m3"]:.3f} m³ ({VOX*100:.0f} cm 복셀)'),Line2D([],[],ls="--",color="#52514e",label="상판 (0.576 × 0.472 m)")]
fig.legend(handles=h,loc="outside upper center",ncol=4,frameon=False)
os.makedirs(os.path.join(V1,"img"),exist_ok=True); fig.savefig(os.path.join(V1,"img","workspace.png"),dpi=150)

# ---- md
L_,Rr,Sh=R["arms"]["left"],R["arms"]["right"],R["shared"]
L=["# 작업공간 분석 결과 (자동 생성 — workspace.py)","",f"샘플 팔당 {N:,}개(관절 범위 균일), 자기 충돌 자세 제외, 베이스 영점(base_link = world), 상판 윗면 z = {PLATE_Z} m. 그림 [../img/workspace.png](../img/workspace.png)","",
   "| 항목 | 왼팔 | 오른팔 |","|---|---|---|",
   f'| 유효 점 (충돌 제외) | {L_["n_valid"]:,} ({L_["n_self_collision"]:,} 제외) | {Rr["n_valid"]:,} ({Rr["n_self_collision"]:,} 제외) |',
   f'| x 범위 [m] | {L_["x"][0]} ~ {L_["x"][1]} | {Rr["x"][0]} ~ {Rr["x"][1]} |',
   f'| y 범위 [m] | {L_["y"][0]} ~ {L_["y"][1]} | {Rr["y"][0]} ~ {Rr["y"][1]} |',
   f'| z 범위 [m] (바닥 기준) | {L_["z"][0]} ~ {L_["z"][1]} | {Rr["z"][0]} ~ {Rr["z"][1]} |',
   f'| **상판 윗면 기준 높이 [m]** | **{L_["above_plate"][0]} ~ {L_["above_plate"][1]}** | {Rr["above_plate"][0]} ~ {Rr["above_plate"][1]} |',
   f'| **최대 전방 도달 x [m]** (그때 z) | **{L_["forward_reach_max"]}** ({L_["forward_reach_z"]}) | {Rr["forward_reach_max"]} ({Rr["forward_reach_z"]}) |',
   f'| 상판 앞 가장자리(x = {PX/2:.3f}) 밖으로 | {L_["forward_beyond_plate_edge"]} | {Rr["forward_beyond_plate_edge"]} |',
   f'| 어깨 중심에서 도달 반경 [m] | {L_["reach_from_shoulder_min"]} ~ {L_["reach_from_shoulder_max"]} | {Rr["reach_from_shoulder_min"]} ~ {Rr["reach_from_shoulder_max"]} |',
   f'| 정중면(y = 0) 넘어 반대쪽으로 [m] | {L_["cross_midline_max"]} | {Rr["cross_midline_max"]} |',
   "","## 높이 구간별 최대 전방 도달 (왼팔, 오른팔 동일)","","| z 구간 [m] | 상판 위 [m] | 점 수 | 최대 x [m] | 상판 앞 가장자리 밖 [m] |","|---|---|---|---|---|"]+[
   f'| {b["z_band"][0]} ~ {b["z_band"][1]} | {b["above_plate"][0]} ~ {b["above_plate"][1]} | {b["n"]:,} | {b["x_max"]} | {b["beyond_plate_edge"]} |' for b in L_["height_bands"]]+[
   "","## 양팔 공유 작업공간",f'- 복셀 {VOX*100:.0f} cm: 왼팔 {Sh["vol_left_m3"]} · 오른팔 {Sh["vol_right_m3"]} · **공유 {Sh["vol_shared_m3"]} m³** (왼팔의 {Sh["shared_pct_of_left"]} %)',
   f'- 공유 영역 범위 x {Sh["x"]} · y {Sh["y"]} · z {Sh["z"]} m (상판 위 {Sh["above_plate"]}), 중심 {Sh["centroid"]}',f'- 정면 양손 목표 후보(x > 0.15): {Sh.get("front_candidates")}',f'- {Sh["note"]}',
   "","## 베이스 이동 포함 시",f'- {R["with_base"]["horizontal"]}; {R["with_base"]["yaw"]}',f'- {R["with_base"]["note"]}',
   "",f"## 토크 예산표 입력 — 도달 가능 자세 중 관절별 정적 토크 최대 (왼팔, [workspace_max_torque.json](workspace_max_torque.json))","",
   "| 페이로드 | 관절 | |τ| 최대 [N·m] | 자세 (pitch, roll, yaw, elbow) ° | ee 위치 [m] | PDR 52 대비 % | A2 정격 대비 % (구동기) |","|---|---|---|---|---|---|---|"]+[
   f'| {pl} | {a} | {v["tau_abs_max"]} | {v["q_deg"]} | {v["ee_pos"]} | {v["margin_vs_pdr52_pct"]} | {v["margin_vs_A2_rated_pct"]} ({v["A2_actuator"]}) |' for pl,o in WT["results"].items() for a,v in o.items()]+[
   "","### PDR 게이트 점검 (기획서: 「액추에이터 토크 여유 20 % 이상」, 목표 사양 「3 kg per arm 최대 신장 자세, 안전율 1.5」)","",
   "| 관절 | 3 kg 최악 |τ| | × 1.2 (PDR 여유) | × 1.5 (안전율) | 팀장 기준 52 | A2 정격 (a2_doc) | 어깨도 RI85면 | 판정 |","|---|---|---|---|---|---|---|---|"]+[
   (lambda v,a: f'| {a} | {v["tau_abs_max"]:.2f} | {v["tau_abs_max"]*1.2:.1f} | {v["tau_abs_max"]*1.5:.1f} | {"OK" if v["tau_abs_max"]*1.2<=RATED_CRIT else "NG"} ({v["margin_vs_pdr52_pct"]} %) | {A2[v["A2_actuator"]]["rated"]} ({"OK" if v["tau_abs_max"]*1.5<=A2[v["A2_actuator"]]["rated"] else "NG"}, {v["margin_vs_A2_rated_pct"]} %) | 72.6 ({"OK" if v["tau_abs_max"]*1.5<=72.6 else "NG"}, {v["margin_vs_RI85_rated_pct"]} %) | {"통과" if v["tau_abs_max"]*1.5<=min(72.6,A2[v["A2_actuator"]]["rated"]) else "주의"} |')(WT["results"]["3kg"][a],a) for a in ARM]+[
   "",f'- 테이블 높이(z 0.7 ~ 0.8 m, 상판 위 0.3 ~ 0.4 m)에서 최대 전방 x {L_["height_bands"][1]["x_max"]} m = 상판 앞 가장자리 밖 {L_["height_bands"][1]["beyond_plate_edge"]} m → 베이스를 테이블 모서리 0.15 m 안까지 붙여야 함',
   "","- A2 정격: 어깨 TQ ILM-E85x30 118.8 N·m, 팔꿈치 RI85 72.6 N·m (사양서 10/19). CAD는 8개 모두 RI85 → 어깨도 RI85라면 json `margin_vs_RI85_rated_pct` 사용",
   f'- 실행 시간 {R["elapsed_s"]} s, MuJoCo {mujoco.__version__}']
open(os.path.join(V1,"results","workspace.md"),"w",encoding="utf-8").write("\n".join(L)+"\n")
print("\n".join(L[4:16])); print("elapsed",R["elapsed_s"])

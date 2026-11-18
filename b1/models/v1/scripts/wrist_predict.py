# week09 박종진 — 손목 2축 예측 분석 (모델 변경 없음 = 기존 v1 4-DOF 팔의 플랜지(tool0) 자세만 쓰고, 손목은 수식으로 덧붙여 계산)
#   가정 손목 W = 「pitch(tool0 x축) → roll(그리퍼 축)」 2축, pitch ±90°, roll ±180°, 같은 그리퍼(tool0 → ee 0.10 m) + 손목 모듈 길이 L_w(0 / 0.06 m 두 경우)
#     · 수직 하강(그리퍼 축 연직 아래) 조건: 4-DOF 지금 = 플랜지 z축 = ẑ (2조건)  →  W = 플랜지 x축 ⟂ ẑ (1조건) 이고 pitch θ = atan2(−a, b) (ẑ_local = (0, a, b), b ≥ 0)
#     · 손가락 닫힘 방향(상자 면): 지금 = 팔 자세에 묶임(+1조건) → W = roll로 자유
#     · 위치 3 + 축 2 + 닫힘 1 = 6조건: 지금 4자유도(팔만) · 7자유도(전신) → W 6자유도(팔만) · 9자유도(전신)
#   방법(7주차 workspace_wb ④와 같음): 무작위 팔 자세 → 조건 위로 Gauss-Newton 투영 → 자기 충돌 없는 자세만 → 끝점(ee) 점 집합 비교
#   출력: results/wrist_predict.{json,md}, img/wrist_predict.png        실행: python b1/models/v1/scripts/wrist_predict.py  (~60 s)
import os,sys,json,time
import numpy as np,mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); ROOT=os.path.dirname(os.path.dirname(os.path.dirname(V1)))
sys.path.insert(0,os.path.join(ROOT,"scenarios")); import pick_env as PE
for _f in ("Malgun Gothic","NanumGothic","Noto Sans CJK KR","AppleGothic"):
    if any(_f in x.name for x in matplotlib.font_manager.fontManager.ttflist): plt.rcParams["font.family"]=_f; break
plt.rcParams["axes.unicode_minus"]=False

ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; N=20000; H_EE=0.10; LW=(0.0,0.06); PITCH_MAX=np.pi/2
HX,HY=0.288,0.236                                   # 상판 반치수(베이스 사각형)
Z_GRASP=PE.TABLE["top"]+0.032+PE.GRIPPER["grasp_off"]   # 파지 때 ee 높이(8주차 GRASP_Z + 0.045) = 0.777 m
m=PE.build(); d=mujoco.MjData(m); bt=m.body("left_tool0").id
jid=[m.joint(f"left_{a}_joint").id for a in ARM]; qa=np.array([m.jnt_qposadr[j] for j in jid]); da=np.array([m.jnt_dofadr[j] for j in jid]); lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
robot={g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name not in ("world","obj_box","obj_cyl") and m.geom(g).name not in ("table","floor")}
zt=np.array([0,0,1.0]); jp=np.zeros((3,m.nv)); jr=np.zeros((3,m.nv))

def fk(q):
    d.qpos[:]=0; d.qpos[qa]=q; mujoco.mj_kinematics(m,d); mujoco.mj_comPos(m,d); return d.xpos[bt].copy(),d.xmat[bt].reshape(3,3).copy()
def self_collide(q):
    d.qpos[:]=0; d.qpos[qa]=q; mujoco.mj_forward(m,d)          # 로봇 geom끼리만 셈(테이블 · 바닥 · 물체 제외)
    for i in range(d.ncon):
        c=d.contact[i]
        if c.geom1 in robot and c.geom2 in robot and c.dist<0: return True
    return False
def project(q,mode,it=40):
    """mode 'now' : 플랜지 z = ẑ (2식)   mode 'wrist' : (플랜지 x)·ẑ = 0 (1식)"""
    for _ in range(it):
        p,R=fk(q); mujoco.mj_jacBody(m,d,jp,jr,bt); W=jr[:,da]     # 각속도 자코비안(world)
        if mode=="now":
            e=np.cross(R[:,2],zt); J=np.array([np.cross(np.cross(W[:,k],R[:,2]),zt) for k in range(4)]).T
            if np.linalg.norm(e)<1e-4 and R[2,2]>0: return q,True
            q=np.clip(q-np.linalg.pinv(J,rcond=1e-6)@e,lo,hi)
        else:
            c=R[:,0]@zt; J=np.array([np.cross(W[:,k],R[:,0])@zt for k in range(4)])
            if abs(c)<1e-5: return q,True
            q=np.clip(q-J*c/(J@J+1e-9),lo,hi)
    p,R=fk(q); return q,(np.linalg.norm(np.cross(R[:,2],zt))<1e-3 and R[2,2]>0) if mode=="now" else abs(R[:,0]@zt)<1e-4

t0=time.time(); rng=np.random.default_rng(9); OUT={}
for mode in ("now","wrist"):
    pts=[]; Q=[]
    for q in rng.uniform(lo,hi,(N,4)):
        q,ok=project(q,mode)
        if not ok: continue
        p,R=fk(q)
        if mode=="wrist":
            a,b=R[:,1]@zt,R[:,2]@zt                     # ẑ를 플랜지 좌표로 = (0, a, b)
            th=np.arctan2(-a,b)
            if abs(th)>PITCH_MAX: continue
        if self_collide(q): continue
        pts.append(p); Q.append(q)
    OUT[mode]=dict(flange=np.array(pts),q=np.array(Q))
    print(mode,len(pts),f"{time.time()-t0:.0f}s",flush=True)

def ee_pts(mode,lw):   # 수직 하강 자세의 ee = 플랜지 − (0.10 + L_w) ẑ (지금 팔은 손목 길이 없음)
    F=OUT[mode]["flange"]; return F-np.array([0,0,H_EE+(lw if mode=="wrist" else 0.0)])

def depth(P,yaw):      # 상판을 테이블 가장자리에 붙였을 때 테이블 안쪽 깊이(7주차 workspace_wb와 같은 식)
    c,s=np.cos(yaw),np.sin(yaw); return c*P[:,0]-s*P[:,1]-(HX*abs(c)+HY*abs(s))
def area_cm2(P,cell=0.02):
    return len({(int(np.floor(x/cell)),int(np.floor(y/cell))) for x,y in P[:,:2]})*cell*cell*1e4

rows=[]; band=0.015
cases=[("4-DOF 지금","now",0.0),("손목 2축 (L_w 0)","wrist",0.0),("손목 2축 (L_w 0.06 m)","wrist",0.06)]
for name,mode,lw in cases:
    P=ee_pts(mode,lw); sel=np.abs(P[:,2]-Z_GRASP)<band; Pg=P[sel]
    rows.append(dict(case=name,n_pose=len(P),z_range=[round(float(P[:,2].min()),3),round(float(P[:,2].max()),3)],
                     n_at_grasp_h=int(sel.sum()),area_at_grasp_h_cm2=round(area_cm2(Pg),0) if len(Pg) else 0,
                     depth_front=round(float(depth(Pg,0).max()),3) if len(Pg) else None,
                     depth_side=round(float(depth(Pg,-np.pi/2).max()),3) if len(Pg) else None))

# 7주차 「위치 + 아래로 접근」 무작위 150목표(wb_ik_compare와 같은 범위 · 개수, 팔만): IK로 직접 — 허용 위치 1 mm · 축 0.01 rad (7주차와 같음)
#   지금 = 플랜지 위치 3 + 축 2 = 5식 / 4미지수(최소제곱, 잔차가 남으면 실패)   W = 플랜지 위치 3 + (x·ẑ) 1 = 4식 / 4미지수 + pitch ±90° · 자기 충돌 점검
def ik_topdown(t,mode,lw,restarts=12,it=60,rng=None):
    f=t+np.array([0,0,H_EE+(lw if mode=="wrist" else 0.0)])
    for q in rng.uniform(lo,hi,(restarts,4)):
        for _ in range(it):
            p,R=fk(q); mujoco.mj_jacBody(m,d,jp,jr,bt); Jp,W=jp[:,da],jr[:,da]
            if mode=="now": r=np.r_[p-f,np.cross(R[:,2],zt)[:2]]; J=np.vstack([Jp,np.array([np.cross(np.cross(W[:,k],R[:,2]),zt) for k in range(4)]).T[:2]])
            else: r=np.r_[p-f,R[:,0]@zt]; J=np.vstack([Jp,[np.cross(W[:,k],R[:,0])@zt for k in range(4)]])
            if np.linalg.norm(r[:3])<1e-3 and np.linalg.norm(r[3:])<1e-2: break
            q=np.clip(q-np.linalg.solve(J.T@J+1e-4*np.eye(4),J.T@r),lo,hi)
        p,R=fk(q); ok=np.linalg.norm(p-f)<1e-3
        ok&=(np.linalg.norm(np.cross(R[:,2],zt))<1e-2 and R[2,2]>0) if mode=="now" else (abs(R[:,0]@zt)<1e-2 and abs(np.arctan2(-(R[:,1]@zt),R[:,2]@zt))<=PITCH_MAX)
        if ok and not self_collide(q): return True
    return False
r2=np.random.default_rng(7); T=np.c_[r2.uniform(0.25,1.0,150),r2.uniform(-0.3,0.8,150),r2.uniform(0.62,1.10,150)]
reach={name:round(100*float(np.mean([ik_topdown(t,mode,lw,rng=np.random.default_rng(i)) for i,t in enumerate(T)])),1) for name,mode,lw in cases}
# 전신(베이스 x · y · yaw 자유): 높이만 맞으면 됨 → 목표 높이가 그 경우의 수직 하강 z 대역 안인 비율(테이블 · 충돌 미고려 상한)
reach_wb={name:round(100*float(np.mean([(ee_pts(mode,lw)[:,2].min()<=t[2]<=ee_pts(mode,lw)[:,2].max()) for t in T])),1) for name,mode,lw in cases}

# 질량 · 토크 영향(가정): 손목 모듈 0.8 kg(소형 서보 2개 + 브래킷 가정)을 어깨에서 0.475 m(상완 0.276 + 전완 0.199), 3 kg 짐 팔 길이 +L_w
TQ_NOW=48.69; g=9.81; MW=0.8; R_W=0.475
tq=[dict(lw=lw,add_wrist=round(MW*g*R_W,2),add_payload=round(3.0*g*lw,2),need=round(TQ_NOW+MW*g*R_W+3.0*g*lw,2),
         pdr52_margin_pct=round(100*(52/(TQ_NOW+MW*g*R_W+3.0*g*lw)-1),1)) for lw in LW]

res=dict(assume=dict(wrist="pitch(tool0 x) ±90° → roll(그리퍼 축) ±180°",H_EE=H_EE,L_w=LW,z_grasp=Z_GRASP,band=band,N=N,wrist_mass_kg=MW,r_wrist=R_W,
                     torque_now_pitch=TQ_NOW),rows=rows,reach_arm_only_pct=reach,reach_wholebody_upper_pct=reach_wb,torque=tq,seconds=round(time.time()-t0,1))
os.makedirs(os.path.join(V1,"results"),exist_ok=True); json.dump(res,open(os.path.join(V1,"results","wrist_predict.json"),"w"),indent=1,ensure_ascii=False)

# 그림: 파지 높이(±15 mm)에서 수직 하강 가능한 끝점(base 기준 xy) — 지금 vs 손목
fig,ax=plt.subplots(1,2,figsize=(10,4.4))
for a_,(name,mode,lw),c in zip(ax,[cases[0],cases[1]],("tab:red","tab:blue")):
    P=ee_pts(mode,lw); Pg=P[np.abs(P[:,2]-Z_GRASP)<band]
    a_.scatter(Pg[:,0],Pg[:,1],s=2,c=c,alpha=0.5); a_.add_patch(plt.Rectangle((-HX,-HY),2*HX,2*HY,fill=False,ec="k"))
    a_.plot([0.288+0.04]*2,[-0.8,0.9],"k--",lw=0.8); a_.text(0.335,0.82,"정면 테이블 가장자리(여유 40 mm)",fontsize=7)
    a_.set_title(f"{name}: 파지 높이 z {Z_GRASP:.3f} m에서 수직 하강 끝점 {len(Pg)}점",fontsize=9); a_.set_aspect("equal"); a_.set_xlim(-0.5,1.0); a_.set_ylim(-0.6,0.95); a_.set_xlabel("x (전방) m"); a_.set_ylabel("y m"); a_.grid(alpha=0.3)
fig.tight_layout(); os.makedirs(os.path.join(V1,"img"),exist_ok=True); fig.savefig(os.path.join(V1,"img","wrist_predict.png"),dpi=130)

L=["# 손목 2축 예측 분석 — 모델 변경 없이 (자동 생성 — `wrist_predict.py`, week09 박종진)","",
   f"가정 손목 = **pitch(tool0 x축, ±90°) → roll(그리퍼 축, ±180°)**, 그리퍼는 지금과 같음(tool0 → ee 0.10 m) + 손목 모듈 길이 L_w 0 / 0.06 m. v1 팔 4관절 무작위 {N:,}자세를 「수직 하강」 조건 위로 투영 · 자기 충돌 제외. 파지 높이 = ee z {Z_GRASP:.3f} m(테이블 0.70 + 8주차 GRASP_Z), 띠 ±{band*1e3:.0f} mm","",
   "| 경우 | 수직 하강 자세 수 | ee 높이 대역 [m] | 파지 높이 자세 수 | 파지 높이 면적 [cm²] (2 cm 격자) | 테이블 깊이 정면 [m] | 테이블 깊이 옆면 [m] |","|---|---|---|---|---|---|---|"]
for r in rows: L.append(f"| {r['case']} | {r['n_pose']:,} | {r['z_range'][0]} ~ {r['z_range'][1]} | {r['n_at_grasp_h']:,} | {r['area_at_grasp_h_cm2']:.0f} | {r['depth_front']} | {r['depth_side']} |")
L+=["","| 7주차 「위치 + 아래로 접근」 무작위 150목표 | "+" | ".join(f"{k}" for k in reach)+" |","|---|"+"---|"*len(reach),
    "| 팔만 IK 수렴률 (재시작 12, 1 mm · 0.01 rad, 자기 충돌 없음) | "+" | ".join(f"{v} %" for v in reach.values())+" |",
    "| 전신 상한 (높이만 대역 안, 테이블 · 충돌 미고려) | "+" | ".join(f"{v} %" for v in reach_wb.values())+" |","",
    "| 토크(가정: 손목 0.8 kg · 어깨에서 0.475 m, 3 kg 짐 팔 길이 +L_w) | 손목 추가 | 짐 팔 길이 추가 | 필요 pitch [N·m] | PDR 52 여유 (기준 ≥ 20 %, 지금 6.8 %) |","|---|---|---|---|---|"]
for t in tq: L.append(f"| L_w {t['lw']} m | +{t['add_wrist']} | +{t['add_payload']} | {t['need']} (지금 {TQ_NOW}) | {t['pdr52_margin_pct']} % |")
open(os.path.join(V1,"results","wrist_predict.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

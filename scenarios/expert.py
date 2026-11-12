# week08 박종진 — 집기 시나리오 상태기계 + 전문가 정책 (전신 IK wb_ik + resolved-rate 궤적)
#   단계: APPROACH → ALIGN → DESCEND → GRASP → LIFT → MOVE → PLACE → RELEASE → RETREAT → RETURN → (판정)
#   · 큰 이동(APPROACH · MOVE · RETURN) = 전신 IK 해(베이스 3 + 팔 4) 사이를 관절공간 5차 궤적
#   · 직선 이동(ALIGN · DESCEND · LIFT · PLACE · RETREAT) = 전신 resolved-rate(6차원: 위치 + 그리퍼 자세 전체, 7자유도 → 영공간 1)
#     4-DOF 팔만으로는 「위치 3 + 축 2 = 5조건」이라 수직 직선을 못 그림(7주차) → 베이스가 같이 미세 이동
#   · 접근 방향 = 위에서 아래(수직 하강), 베이스 옆면(yaw ≈ −90°)으로 테이블에 붙임 (7주차 workspace_wb ④, 함태훈 11/05 합의)
#   · 손가락 닫힘 방향 = 상자 면에 수직(정렬 단계, 상자 대칭 4방향 중 IK가 가장 가까운 것), 원기둥은 IK 자연 해
#   · 제어 주기 50 Hz(시뮬 500 Hz), 팔 = position 목표(기준 궤적 적분), 베이스 = velocity(기준 속도 + 위치 피드백), 그리퍼 = 벌림 목표
#   · 행동(action, 8) = [베이스 vx · vy · wz (world), 왼팔 pitch · roll · yaw · elbow 목표, 그리퍼 벌림 목표] — 오른팔은 영점 고정
#   실패 분류: ik_fail · table_collision · approach_timeout · align_fail · grasp_miss · slip · drop · place_miss · tipped · sim_unstable(수치 발산)
import os,sys
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from pick_env import (build,reset,sample_task,contacts,Ids,TABLE,GOAL,GRIPPER,OBJ,obj_half_h)
from wb_ik import (wb_jacobian,wb_weights,wdls,null_proj,limit_avoid,qadr,_clip_base,solve_wb_ik,ARM)

DT_SIM=0.0005; NSUB=40; DT=DT_SIM*NSUB         # 제어 50 Hz, 시뮬 2 kHz (함태훈 week08 권장 dt 0.5 ms)
GRIP_RATE=0.06                                   # 그리퍼 명령 변화율 [m/s] — 한 번에 닫으면 손가락 1.5 m/s로 물체를 쳐서 관통 · 발산(8주차 발견)
GRASP_Z=TABLE["top"]+0.032                      # 파지 중심 높이(물체 아래 1/2 지점 근처, 손가락 끝 테이블 위 12 mm)
PRE_H=0.10                                      # 접근 · 들기 높이 [m]
CLEAR,CLEAR_PAD,CLEAR_PATH=0.02,0.005,0.05      # 테이블 여유 거리(계획 점검): 정지 자세 20 mm(손가락 파지 자세 5 mm), 관절공간 이동 경로 50 mm
CLEAR_BASE=0.02                                 # 베이스 상판 · 하체는 일부러 테이블에 붙임(TABLE_X 여유 40 mm) → 20 mm
TABLE_X=TABLE["edge_x"]-0.04                    # 베이스 상판이 테이블을 뚫지 않게(여유 40 mm = 속도 제어 추종 오버슈트 ~20 mm + 여유)
BASE_BOX=((-0.6,TABLE_X),(-0.9,0.9))
TAU_B=np.array([0.375,0.375,0.082])            # 베이스 속도 응답 시간상수(함태훈 week05 측정 τ 0.380 · yaw 0.082 s) → 가속도 앞먹임으로 지연 보상
W_B=(20.0,20.0)                                 # 베이스 가중치(xy · yaw) — 팔 우선
STAGES=["APPROACH","ALIGN","DESCEND","GRASP","LIFT","MOVE","PLACE","RELEASE","RETREAT","RETURN","DONE"]
FAIL=["ik_fail","table_collision","approach_timeout","align_fail","grasp_miss","slip","drop","place_miss","tipped","sim_unstable"]

def quintic(s):
    s=np.clip(s,0,1); return 10*s**3-15*s**4+6*s**5, (30*s**2-60*s**3+30*s**4)
def rot_des(xc):
    """그리퍼 목표 자세: ee z = +ẑ(접근 −z = 연직 아래), ee x = 손가락 닫힘 방향(수평)"""
    xc=np.array([xc[0],xc[1],0.0]); xc/=np.linalg.norm(xc); z=np.array([0,0,1.0]); return np.column_stack([xc,np.cross(z,xc),z])
def rot_err(R,Rd):
    """R → Rd 회전 오차(world 축 · 각 벡터)"""
    Re=Rd@R.T; q=np.zeros(4); mujoco.mju_mat2Quat(q,Re.ravel()); v=np.zeros(3); mujoco.mju_quat2Vel(v,q,1.0); return v

class Ref:
    """기준 상태(베이스 3 + 왼팔 4) 기구학 — 별도 MjData(물리와 분리)"""
    def __init__(s,m):
        s.m=m; s.d=mujoco.MjData(m); s.QA=qadr(m,"left",True); s.sid=m.site("left_ee").id
        jid=[m.joint(f"left_{a}_joint").id for a in ARM]; s.lo,s.hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
        s.W=wb_weights(True,*W_B)
    def fk(s,x):
        s.d.qpos[s.QA]=x; mujoco.mj_kinematics(s.m,s.d); mujoco.mj_comPos(s.m,s.d)
        return s.d.site_xpos[s.sid].copy(),s.d.site_xmat[s.sid].reshape(3,3).copy()
    def rr_step(s,x,p_des,v_des,Rd,kp=8.0,kr=6.0,lam=0.02,k_limit=0.3):
        """resolved-rate 1주기: q̇ = J#_W [v + Kp e_p ; Kr e_r] + N q̇0  → (x_new, q̇)"""
        p,R=s.fk(x); (Jp,Jr),_=wb_jacobian(s.m,s.d,"left",True,rot=True); J=np.vstack([Jp,Jr])
        xd=np.concatenate([v_des+kp*(p_des-p),kr*rot_err(R,Rd)])
        q0=limit_avoid(s.m,s.d,"left",True,k_limit); q0[:3]=0
        qd=wdls(J,s.W,lam)@xd+null_proj(J,s.W)@q0
        qd[:2]=np.clip(qd[:2],-0.6,0.6); qd[2]=np.clip(qd[2],-1.0,1.0); qd[3:]=np.clip(qd[3:],-2.0,2.0)
        xn=x+qd*DT; xn[3:]=np.clip(xn[3:],s.lo,s.hi); xn[:3]=_clip_base(xn[:3],BASE_BOX,None,TABLE_X)
        return xn,(xn-x)/DT        # 클리핑 후 실제 기준 속도(베이스 앞먹임이 한계 밖으로 밀지 않게)

    def pose_ik(s,x0,p_des,Rd,iters=1500,tol_p=1e-3,tol_r=np.deg2rad(1.0)):
        """전신 자세 IK(위치 3 + 자세 3 = 6조건, 7자유도) — rr_step을 수렴까지 반복(가상 시간)"""
        x=x0.copy()
        for k in range(iters):
            p,R=s.fk(x); ep=np.linalg.norm(p_des-p); er=np.linalg.norm(rot_err(R,Rd))
            if ep<tol_p and er<tol_r: return x,True,ep,er
            x,_=s.rr_step(x,p_des,np.zeros(3),Rd,kp=1/DT*0.5,kr=1/DT*0.5,lam=0.03)
        p,R=s.fk(x); return x,False,float(np.linalg.norm(p_des-p)),float(np.linalg.norm(rot_err(R,Rd)))

    def table_hits(s,x,I,clear=CLEAR,pad_clear=CLEAR_PAD):
        """기준 자세 x에서 로봇 geom ↔ 테이블 최소 거리 < 여유(clear)인 geom 목록 (계획 단계 점검, mj_geomDistance).
           여유: 손가락 5 mm(파지 자세에서 손끝이 테이블 위 12 mm라서), 나머지(팔 링크 · 손바닥 · 베이스) 20 mm — 실제 추종 오차 흡수"""
        s.d.qpos[s.QA]=x; mujoco.mj_kinematics(s.m,s.d)
        out=[]
        for g in I.robot:
            c=pad_clear if g in I.pads else (min(clear,CLEAR_BASE) if s.m.geom(g).name.startswith("base_") else clear)
            if mujoco.mj_geomDistance(s.m,s.d,g,I.table,c,None)<c: out.append(s.m.geom(g).name)
        return sorted(out)

_LIB=None
def down_library(m):
    """수직 하강 가능 팔 자세 모음(7주차 workspace_wb ④와 같은 방법: 무작위 → 축 연직 Gauss-Newton 투영, 자기 충돌 없음).
       전신 IK가 정면 근처 국소해(yaw ≈ −100°, 깊이 ≈ 0.16 m)에 갇히는 문제 → 옆으로 멀리 뻗은 자세를 시작값으로 씀. 캐시 results/down_poses.npz"""
    global _LIB
    if _LIB is not None: return _LIB
    f=os.path.join(HERE,"results","down_poses.npz")
    if os.path.exists(f): z=np.load(f); _LIB=(z["q"],z["p"]); return _LIB
    dd=mujoco.MjData(m); QA=qadr(m,"left",True); sid=m.site("left_ee").id
    jid=[m.joint(f"left_{a}_joint").id for a in ARM]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]; DA=np.array([m.jnt_dofadr[j] for j in jid])
    rng=np.random.default_rng(3); jr=np.zeros((3,m.nv)); zt=np.array([0,0,1.0]); Q=[]; P=[]
    for q in rng.uniform(lo,hi,(20000,4)):
        for it in range(25):
            dd.qpos[:]=0; dd.qpos[QA[3:]]=q; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd)
            zc=dd.site_xmat[sid].reshape(3,3)[:,2]; e=np.cross(zc,zt)
            if np.linalg.norm(e)<1e-4 and zc@zt>0: break
            mujoco.mj_jacSite(m,dd,None,jr,sid); Pz=np.eye(3)-np.outer(zc,zc)
            q=np.clip(q+np.linalg.pinv(Pz@jr[:,DA],rcond=1e-6)@(Pz@e),lo,hi)
        if np.linalg.norm(e)<1e-3 and zc@zt>0: Q.append(q.copy()); P.append(dd.site_xpos[sid].copy())
    os.makedirs(os.path.dirname(f),exist_ok=True); np.savez_compressed(f,q=np.array(Q),p=np.array(P)); _LIB=(np.array(Q),np.array(P)); return _LIB

def library_seeds(m,target,n=3,q_ref=None):
    """옆면(yaw −90°) 붙이기 기준 시작값: 베이스 x = 한계, 필요한 옆 도달 y_l = tx − x_b, 높이 맞는 자세 중 y_l이 필요값 이상에서 가장 작은 것"""
    Q,P=down_library(m); psi=-np.pi/2; xb=TABLE_X-0.236-0.005
    need=target[0]-xb; s=np.where((np.abs(P[:,2]-target[2])<0.015)&(P[:,1]>=need+0.003))[0]
    s=s[np.argsort(P[s,1])][:n] if q_ref is None else s[np.argsort(np.linalg.norm(Q[s]-q_ref,axis=1))][:n]   # 놓기: 지금 팔 자세와 가까운 가지
    return [np.concatenate([[xb,target[1]+P[i,0],psi],Q[i]]) for i in s]

def plan_grasp_pose(ref,m,x_now,pos_g,obj_yaw,kind,I,seed=0,q_ref=None):
    """파지 자세(손가락 사이 = 물체) 먼저 → 그 위 PRE_H 접근 자세. 수직 하강 중 베이스가 테이블 쪽으로 움직여야 하므로(4-DOF)
       한계에 붙은 접근 자세에서 시작하면 하강을 못 끝냄 → 파지 자세를 테이블 제한 안에서 먼저 풀고 위로 올림.
       ① 축만 전신 IK(wb_ik, 옆면 yaw 시드) → 자연스러운 손가락 방향 ② 상자면 면 법선 4방향 중 가장 가까운 것 ③ 자세 전체 IK
       ④ 두 자세 모두 테이블 충돌 점검(손가락 · 팔 링크 · 베이스) — 충돌이면 다음 시드"""
    eg=np.array([pos_g[0],pos_g[1],GRASP_Z+GRIPPER["grasp_off"]]); ep_=eg+np.array([0,0,PRE_H]); dd=mujoco.MjData(m); dd.qpos[ref.QA]=x_now
    tries=[]
    starts=[("lib",x) for x in library_seeds(m,eg,n=8,q_ref=q_ref)]+[("wb",b) for b in (-np.pi/2,-np.pi/2+0.4,-np.pi/2-0.4)]
    for kind_s,b_seed in starts:
        if kind_s=="lib": x=b_seed; st="library"; b_seed=x[2]
        else:
            base0=np.array([min(x_now[0],TABLE_X-0.25),pos_g[1],b_seed])
            r=solve_wb_ik(m,dd,"left",eg,[0,0,-1],base_init=base0,w_xy=1.0,w_yaw=1.0,table_x=TABLE_X,base_box=BASE_BOX,restarts=6,seed=seed)
            x=np.concatenate([r.base,r.q]); st=r.status
        _,R=ref.fk(x); xc=R[:,0]
        if kind=="box":   # 상자 면 법선 4방향(대칭) — IK 자연 방향에 가까운 순서로 시도
            c=[np.array([np.cos(obj_yaw+k*np.pi/2),np.sin(obj_yaw+k*np.pi/2)]) for k in range(4)]
            xcs=[c[i] for i in np.argsort([-(v@xc[:2]) for v in c])][:2]
        else: xcs=[xc[:2]]
        for xc2 in xcs:
            Rd=rot_des(xc2); xg,okg,eg_p,eg_r=ref.pose_ik(x,eg,Rd); okp=False; hits=[]
            if okg:
                for ph in (PRE_H,0.07,0.05):   # 깊은 물체: 접근 높이를 낮춤(위로 갈수록 베이스가 테이블 쪽으로 가야 함)
                    xp,okp,ep_p,ep_r=ref.pose_ik(xg,eg+np.array([0,0,ph]),Rd)
                    if okp: break
                hits=sorted(set(ref.table_hits(xg,I))|set(ref.table_hits(xp,I,pad_clear=CLEAR))) if okp else []
            tries.append(dict(start=kind_s,base_seed_deg=round(float(np.rad2deg(b_seed))),wb_status=st,grasp_ok=okg,pre_ok=okp,pre_h=ph if okg else None,
                              pos_err_mm=round(eg_p*1e3,2),rot_err_deg=round(float(np.rad2deg(eg_r)),2),base_yaw_deg=round(float(np.rad2deg(xg[2])),1),table_hits=hits))
            tries[-1]["pose_ok"]=okg and okp
            if okg and okp and not hits: tries[-1]["chosen"]=True; return xp,Rd,tries
    # 파지 자세에서 위로 이어서 못 풀면(깊은 물체: 파지 자세 가지에서 위로 가면 팔 관절 한계): 접근 자세를 라이브러리 시작값으로 따로 풀고
    # 하강(resolved-rate)을 계획 단계에서 모의해 파지 자세에 닿는지 확인 → 실행과 같은 식이라 그대로 재현됨
    for xs in library_seeds(m,ep_,n=4):
        _,R=ref.fk(xs); xc=R[:,0]
        if kind=="box":
            c=[np.array([np.cos(obj_yaw+k*np.pi/2),np.sin(obj_yaw+k*np.pi/2)]) for k in range(4)]; xc=c[int(np.argmax([v@xc[:2] for v in c]))]
        Rd=rot_des(xc[:2]); xp,okp,_,_=ref.pose_ik(xs,ep_,Rd)
        if not okp: continue
        xe=simulate_line(ref,xp,eg,Rd,1.6); pe2,_=ref.fk(xe); okd=bool(np.linalg.norm(pe2-eg)<0.003)
        hits=sorted(set(ref.table_hits(xe,I))|set(ref.table_hits(xp,I,pad_clear=CLEAR))) if okd else []
        tries.append(dict(start="lib_pre",grasp_ok=okd,pre_ok=okp,pre_h=PRE_H,pos_err_mm=round(float(np.linalg.norm(pe2-eg))*1e3,2),table_hits=hits,pose_ok=okd))
        if okd and not hits: tries[-1]["chosen"]=True; return xp,Rd,tries
    return None,None,tries

def simulate_line(ref,x,p1,Rd,T):
    """계획 단계에서 직선 구간(resolved-rate, 실행과 같은 식)을 기준 기구학으로 모의 → 끝 상태"""
    p0,_=ref.fk(x); n=int(round(T/DT))
    for k in range(n):
        a,da=quintic((k+1)*DT/T); x,_=ref.rr_step(x,p0+(p1-p0)*a,(p1-p0)*da/T,Rd)
    for k in range(25): x,_=ref.rr_step(x,p1,np.zeros(3),Rd)
    return x

def plan_place_pose(ref,m,x_now,Rd_now,I,seed=0):
    """목표 영역 위 자세. ① 「평행이동」 시작값: 지금 팔 자세 그대로 베이스만 목표 쪽으로 옮김 → 자세 IK(같은 그리퍼 자세)
       → 이동 구간에서 팔이 거의 안 바뀜(물체를 흔들지 않음) ② 안 되면 파지 계획과 같은 방법(연직축 둘레 자유)"""
    g=np.array([TABLE["edge_x"]+GOAL["depth"],GOAL["y"]]); el=np.array([g[0],g[1],GRASP_Z+GRIPPER["grasp_off"]])
    pe,_=ref.fk(x_now); ep_=el+np.array([0,0,max(pe[2]-el[2],0.05)]); x0=x_now.copy(); x0[0]+=g[0]-pe[0]; x0[1]+=g[1]-pe[1]; x0[:3]=_clip_base(x0[:3],BASE_BOX,None,TABLE_X)
    xp,okp,e2,r2=ref.pose_ik(x0,ep_,Rd_now)                       # 같은 높이로 평행이동(거의 그대로)
    xl,okl,e1,r1=ref.pose_ik(xp,el,Rd_now) if okp else (xp,False,9,9)   # 거기서 놓는 높이로
    hits=sorted(set(ref.table_hits(xl,I))|set(ref.table_hits(xp,I,pad_clear=CLEAR))) if (okl and okp) else []
    t0=dict(start="translate",grasp_ok=okl,pre_ok=okp,pose_ok=okl and okp,pos_err_mm=round(e1*1e3,2),base_yaw_deg=round(float(np.rad2deg(xl[2])),1),table_hits=hits)
    if okl and okp and not hits: return xp,Rd_now,[t0]
    x,Rd,tries=plan_grasp_pose(ref,m,x_now,g,0.0,"cyl",I,seed,q_ref=x_now[3:])
    return x,Rd,[t0]+tries

BACKOFF=0.30   # 접근 · 복귀 경유점: 베이스를 테이블에서 world −x로 0.30 m 뺀 자리에서 팔 자세를 바꿈(관절 보간 중 손이 테이블을 쓸지 않게)
def via_backoff(x,bo=BACKOFF):
    w=x.copy(); w[0]-=bo; w[:3]=_clip_base(w[:3],BASE_BOX,None,TABLE_X); return w

def path_hits(ref,I,x0,x1,n=30):
    """관절공간 직선(5차 보간과 같은 경로) 위 n점에서 로봇 ↔ 테이블 충돌 geom 목록"""
    out=set()
    for a in np.linspace(0,1,n): out|=set(ref.table_hits(x0+(x1-x0)*a,I,clear=CLEAR_PATH,pad_clear=CLEAR_PATH))   # 이동 경로: 서보 추종 지연(손이 기준보다 낮게 지나감) → 50 mm
    return sorted(out)

def seg_time(x0,x1):
    """관절공간 구간 시간: 5차 최대 가속 5.77 D/T² ≤ 베이스 1.0 m/s² · 1.57 rad/s²(7주차 가속 제한), 팔 최대 속도 1.0 rad/s"""
    db=np.linalg.norm(x1[:2]-x0[:2]); dy=abs(x1[2]-x0[2]); da=np.max(np.abs(x1[3:]-x0[3:]))
    return float(max(1.5,np.sqrt(5.77*db/0.8),np.sqrt(5.77*dy/1.2),1.875*da/1.0))

class Expert:
    """상태기계. step(m, d) 호출마다(50 Hz) 다음 ctrl을 d.ctrl에 쓰고 action(8)을 돌려줌. done이면 self.result"""
    def __init__(s,m,d,task,I):
        s.m,s.I,s.task=m,I,task; s.ref=Ref(m); s.QA=s.ref.QA
        s.x=d.qpos[s.QA].copy(); s.x_home=s.x.copy(); s.xd=np.zeros(7); s.xdd=np.zeros(7); s.grip=s.grip_tgt=GRIPPER["stroke"]
        s.stage="APPROACH"; s.t=0.0; s.t_total=0.0; s.seg=None; s.fail=None; s.info=dict(stage_t={},table_contacts=[],plan={})
        s.held_once=False; s.pre_h=PRE_H; s.queue=[]; s.t_seg0=0.0; s.done=False; s.result=None; s.ok=s.obj_kind=task["kind"]
        s.ob=I.obj[task["kind"]]; s.Rd=None; s.events=[]
        s._enter("APPROACH",d)

    # ---- 측정(시뮬 참값 = 완전 관측 가정) ----
    def obj_pose(s,d):
        p=d.xpos[s.ob].copy(); R=d.xmat[s.ob].reshape(3,3); yaw=float(np.arctan2(R[1,0],R[0,0])); tilt=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1))))
        return p,yaw,tilt
    def ee(s,d): return d.site_xpos[s.I.ee].copy(),d.site_xmat[s.I.ee].reshape(3,3).copy()

    def _joint_seg(s,*xs):
        """관절공간 구간들(경유점 포함). 계획 단계에서 경로 위 테이블 충돌 점검 → info.path_hits"""
        s.queue=list(xs); x0=s.x.copy(); hits=[]
        for x1 in xs: hits+=path_hits(s.ref,s.I,x0,x1); x0=x1
        s.info.setdefault("path_checks",[]).append((s.stage,sorted(set(hits))))
        s._next_seg()
    def _next_seg(s):
        x1=s.queue.pop(0); s.seg=("joint",s.x.copy(),x1,seg_time(s.x,x1)); s.t_seg0=s.t
    def _line_seg(s,p0,p1,T):
        s.seg=("line",p0,p1,T)
    def _enter(s,st,d):
        s.info["stage_t"][s.stage]=round(s.t,2); s.stage=st; s.t=0.0; s.events.append((round(s.t_total,2),st))
        p,yaw,_=s.obj_pose(d)
        if st=="APPROACH":
            x1,Rd,tries=plan_grasp_pose(s.ref,s.m,s.x,p[:2],yaw,s.task["kind"],s.I,seed=s.task["seed"])
            s.info["plan"]["grasp"]=tries
            if x1 is None: return s._fail("ik_fail" if not any(t["pose_ok"] for t in tries) else "table_collision",d)
            s.Rd=Rd; s.pre_h=next(t["pre_h"] for t in tries if t.get("chosen"))
            for bo in (0.30,0.45,0.60,0.75):   # 경유점 거리: 경로 위 테이블 충돌이 없는 가장 가까운 것
                w=via_backoff(x1,bo); w0=s.x.copy(); w0[:3]=w[:3]
                h=path_hits(s.ref,s.I,s.x,w0)+path_hits(s.ref,s.I,w0,w)+path_hits(s.ref,s.I,w,x1)
                if not h: break
            s.info["approach_backoff"]=bo
            if h: return s._fail("table_collision",d)
            s._joint_seg(w0,w,x1)    # ① 베이스만 경유점으로(팔 내린 채) ② 팔 자세 바꾸기(테이블에서 떨어져서) ③ 베이스만 테이블로
        elif st=="ALIGN":   # 물체를 다시 측정해 접근점 보정(직선 0.6 s)
            pe,_=s.ref.fk(s.x); tgt=np.array([p[0],p[1],GRASP_Z+GRIPPER["grasp_off"]+s.pre_h]); s._line_seg(pe,tgt,0.6)
        elif st=="DESCEND":
            pe,_=s.ref.fk(s.x); s._line_seg(pe,np.array([p[0],p[1],GRASP_Z+GRIPPER["grasp_off"]]),1.6)
        elif st=="GRASP": s.seg=("hold",); s.grip_tgt=0.0   # 명령은 GRIP_RATE로 서서히(손가락이 물체를 치지 않게)
        elif st=="LIFT":
            pe,_=s.ref.fk(s.x); s._line_seg(pe,pe+np.array([0,0,s.pre_h]),1.6)
        elif st=="MOVE":
            x1,Rd,tries=plan_place_pose(s.ref,s.m,s.x,s.Rd,s.I,s.task["seed"]); s.info["plan"]["place"]=tries
            if x1 is None: return s._fail("ik_fail" if not any(t["pose_ok"] for t in tries) else "table_collision",d)
            if path_hits(s.ref,s.I,s.x,x1): return s._fail("table_collision",d)
            s.Rd=Rd; s._joint_seg(x1)
        elif st=="PLACE":
            pe,_=s.ref.fk(s.x); s._line_seg(pe,np.array([pe[0],pe[1],GRASP_Z+GRIPPER["grasp_off"]-0.004]),1.6)   # 물체가 테이블에 닿으면 바로 RELEASE
        elif st=="RELEASE": s.seg=("hold",); s.grip_tgt=GRIPPER["stroke"]
        elif st=="RETREAT":
            pe,_=s.ref.fk(s.x); s._line_seg(pe,pe+np.array([0,0,PRE_H]),1.2)
        elif st=="RETURN":
            for bo in (0.30,0.45,0.60):
                w=via_backoff(s.x,bo); w2=s.x_home.copy(); w2[:3]=w[:3]
                if not (path_hits(s.ref,s.I,s.x,w)+path_hits(s.ref,s.I,w,w2)): break
            s._joint_seg(w,w2,s.x_home)
        elif st=="DONE": s.seg=("hold",)

    def _fail(s,why,d):
        s.fail=why; s.events.append((round(s.t_total,2),"FAIL:"+why)); s.stage_fail=s.stage; s.stage="DONE"; s.seg=("hold",); s.t=0.0

    # ---- 기준 갱신 + 명령 ----
    def _advance_ref(s):
        sg=s.seg; xd_prev=s.xd.copy(); s.xd[:]=0
        if sg[0]=="joint":
            _,x0,x1,T=sg; tt=s.t-s.t_seg0
            if tt>=T and s.queue: s._next_seg(); _,x0,x1,T=s.seg; tt=0.0
            a,da=quintic((tt+DT)/T); xn=x0+(x1-x0)*a; s.xd=(x1-x0)*da/T; s.x=xn
        elif sg[0]=="line":
            _,p0,p1,T=sg; a,da=quintic((s.t+DT)/T); pd=p0+(p1-p0)*a; vd=(p1-p0)*da/T
            s.x,s.xd=s.ref.rr_step(s.x,pd,vd,s.Rd)
        s.xdd=(s.xd-xd_prev)/DT
    def _ctrl(s,d):
        A=s.I.act; qb=d.qpos[s.QA[:3]]
        s.grip+=np.clip(s.grip_tgt-s.grip,-GRIP_RATE*DT,GRIP_RATE*DT)
        vb=s.xd[:3]+TAU_B*s.xdd[:3]+2.0*(s.x[:3]-qb)   # v = v_ref + τ a_ref + K e (1차 지연 보상)
        vb[:2]=np.clip(vb[:2],-1.5,1.5); vb[2]=np.clip(vb[2],-1.5708,1.5708)
        act=np.concatenate([vb,s.x[3:],[s.grip]])
        apply_action(s.m,d,act,A); return act

    def _hold_here(s,d):
        """기준을 현재 측정 자세로 고정(접촉 후 더 누르지 않게)"""
        s.x[3:]=d.qpos[s.QA[3:]]; s.xd[:]=0
    def _seg_done(s):
        if s.seg[0]=="hold": return True
        if s.seg[0]=="joint": return (not s.queue) and s.t-s.t_seg0>=s.seg[-1]
        return s.t>=s.seg[-1]
    def _over(s):   # 마지막 구간 끝난 뒤 경과 시간
        return s.t-(s.t_seg0 if s.seg[0]=="joint" else 0.0)-s.seg[-1]

    def step(s,d):
        """상태 전환 판정(측정 기반) → 기준 갱신 → ctrl 쓰기. 반환 action(8)"""
        I=s.I
        W_=mujoco.mjtWarning
        if s.stage!="DONE" and any(d.warning[w].number>0 for w in (W_.mjWARN_BADQACC,W_.mjWARN_BADQVEL,W_.mjWARN_BADQPOS)):
            s._fail("sim_unstable",d)   # 수치 발산(MuJoCo가 상태를 리셋함) — 다른 실패로 잘못 분류되지 않게 먼저 판정
        p,yaw,tilt=s.obj_pose(d); pe,Re=s.ee(d); C=contacts(s.m,d,I)
        hits=[g for g in C["robot_table"]]
        if hits and s.stage!="DONE":
            s.info["table_contacts"].append((round(s.t_total,2),s.stage,sorted(set(hits))))
            if any(not g.startswith("left_pad") for g in hits) or s.stage in ("APPROACH","MOVE","RETURN"): s._fail("table_collision",d)
        st=s.stage; held=len(C["pad_obj"])==2; T=s.t
        if st=="APPROACH" and s._seg_done():
            xr,_=s.ref.fk(s.x); err=np.linalg.norm(pe-xr)
            if err<0.015: s._enter("ALIGN",d)
            elif s._over()>2.0: s._fail("approach_timeout",d)
        elif st=="ALIGN" and s._seg_done():
            tgt=np.array([p[0],p[1]]); exy=np.linalg.norm(pe[:2]-tgt); erot=np.degrees(np.linalg.norm(rot_err(Re,s.Rd)))
            if exy<0.006 and erot<4.0: s.info["align_err_mm"]=round(exy*1e3,2); s._enter("DESCEND",d)
            elif T>s.seg[-1]+1.5: s.info["align_err_mm"]=round(exy*1e3,2); s._fail("align_fail",d)
        elif st=="DESCEND":
            if C["palm_obj"] or C["robot_obj_other"]: s._fail("grasp_miss",d)   # 손바닥이 물체 위를 누름 = 정렬 · 높이 오차
            elif s._seg_done() and (abs(pe[2]-(GRASP_Z+GRIPPER["grasp_off"]))<0.006 or T>s.seg[-1]+1.0): s._enter("GRASP",d)
        elif st=="GRASP":
            w=float(np.mean(d.qpos[I.fq])); v=float(np.max(np.abs(d.qvel[[s.m.jnt_dofadr[s.m.joint(f'left_finger_{n}_joint').id] for n in 'lr']])))
            if T>1.0 and held and v<0.005: s.info["grip_width_mm"]=round(2*w*1e3,1); s._enter("LIFT",d)
            elif T>1.8 or (T>0.5 and w<0.012): s.info["grip_width_mm"]=round(2*w*1e3,1); s._fail("grasp_miss",d)
        elif st=="LIFT":
            if s._seg_done():
                rise=p[2]-(TABLE["top"]+obj_half_h(s.task["kind"]))
                if held and rise>0.7*s.pre_h: s.held_once=True; s.info["lift_rise_mm"]=round(rise*1e3,1); s._enter("MOVE",d)
                elif T>s.seg[-1]+0.5: s.info["lift_rise_mm"]=round(rise*1e3,1); s._fail("slip",d)
        elif st=="MOVE":
            if np.linalg.norm(p-(pe-np.array([0,0,GRIPPER["grasp_off"]])) )>0.04 or not held: s._fail("drop",d)
            elif s._seg_done():
                xr,_=s.ref.fk(s.x)
                if np.linalg.norm(pe-xr)<0.012: s._enter("PLACE",d)
                elif s._over()>3.0: s._fail("approach_timeout",d)
        elif st=="PLACE":
            if not held and p[2]<TABLE["top"]+0.06 and s.t<s.seg[-1]*0.5: s._fail("drop",d)
            elif C["obj_table"] and T>0.3: s.info["place_contact_t"]=round(T,2); s._hold_here(d); s._enter("RELEASE",d)
            elif s._seg_done() and s._over()>0.5: s._enter("RELEASE",d)
        elif st=="RELEASE":
            if T>0.6 and s.grip>=GRIPPER["stroke"]-1e-6: s._enter("RETREAT",d)
        elif st=="RETREAT" and s._seg_done(): s._enter("RETURN",d)
        elif st=="RETURN" and s._seg_done() and T>s.seg[-1]+0.4: s._judge(d);
        elif st=="DONE" and not s.done and T>0.4: s._judge(d)
        if not s.done: s._advance_ref()
        act=s._ctrl(d); s.t+=DT; s.t_total+=DT; return act

    def _judge(s,d):
        """성공 판정: 물체가 목표 원 안(xy ≤ tol) · 테이블 위에 놓임(높이 ±10 mm) · 기울기 < 15° · 들어 올린 적 있음 · 실패 이벤트 없음"""
        p,yaw,tilt=s.obj_pose(d); g=np.array([TABLE["edge_x"]+GOAL["depth"],GOAL["y"]]); exy=float(np.linalg.norm(p[:2]-g))
        on=abs(p[2]-(TABLE["top"]+obj_half_h(s.task["kind"])))<0.01
        why=s.fail
        if why is None:
            if tilt>15: why="tipped"
            elif exy>GOAL["tol"] or not on: why="place_miss"
        s.done=True; s.stage="DONE"
        s.result=dict(seed=s.task["seed"],kind=s.task["kind"],mass=round(s.task["mass"],3),success=why is None,fail=why,
                      fail_stage=getattr(s,"stage_fail",None),goal_err_mm=round(exy*1e3,1),tilt_deg=round(tilt,1),
                      t_total=round(s.t_total,2),events=s.events,**{k:v for k,v in s.info.items() if k!="stage_t"})

def apply_action(m,d,act,A=None):
    """action(8) → d.ctrl (13): 베이스 3 · 왼팔 4 · 오른팔 영점 · 손가락 2(같은 값). 재생에서도 같은 함수"""
    if A is None: A={m.actuator(i).name:i for i in range(m.nu)}
    d.ctrl[A["base_vx"]],d.ctrl[A["base_vy"]],d.ctrl[A["base_wz"]]=act[0],act[1],act[2]
    for k,a in enumerate(ARM): d.ctrl[A[f"left_{a}"]]=act[3+k]; d.ctrl[A[f"right_{a}"]]=0.0
    d.ctrl[A["left_finger_l"]]=d.ctrl[A["left_finger_r"]]=act[7]

def run_episode(m,seed,max_t=40.0,on_step=None):
    """한 에피소드: seed → reset → 상태기계 → result. on_step(d, act, expert)로 기록 · 렌더"""
    d=mujoco.MjData(m); task=sample_task(seed); I=reset(m,d,task); ex=Expert(m,d,task,I)
    n=0
    while not ex.done and n*DT<max_t:
        act=ex.step(d)
        if on_step: on_step(d,act,ex)
        for _ in range(NSUB): mujoco.mj_step(m,d)
        n+=1
    if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
    return ex.result,d

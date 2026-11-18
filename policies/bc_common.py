# week09 박종진 — 행동 복제(BC) 공통: 관측 특징 · 행동 목표 정의, HDF5 로더, 실시간 관측(MjData → 같은 특징)
#   입력 형식 = datasets/dataset_spec.md v1.0 (함태훈 b1-dataset-1.0). 학습 · 롤아웃이 같은 함수(features)로 특징을 만든다 → 학습/평가 불일치 없음
#   특징(39) = 로봇(베이스) 좌표 기준. stage(상태기계 단계)는 명세 §2대로 학습 입력에 넣지 않음
#   출력(행동 목표) 2종 — 9주차 「관절 vs 끝점 출력 비교」
#     joint : [action_base_body 3, 팔 관절 목표 − 현재 팔 관절 4, 그리퍼 벌림 목표 1]                    = 8
#     ee    : [action_base_body 3, 끝점 목표 위치 − 현재 끝점 (로봇 좌표) 3, 끝점 목표 z축 (로봇 좌표) 3, 그리퍼 1] = 10
#             끝점 목표 = 그 주기 관절 목표를 그 순간 베이스 pose에서 FK (함태훈 week08 action_format.py와 같은 정의) → 롤아웃에서 팔 IK(DLS, 직전 해에서 시작)
import os,sys,glob,json
import numpy as np,mujoco,h5py
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,os.path.join(ROOT,"scenarios"))
import pick_env as P

ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
GOAL_W=np.array([P.TABLE["edge_x"]+P.GOAL["depth"],P.GOAL["y"],P.TABLE["top"]])   # 목표 원 중심(world, 테이블 윗면)
FEAT_NAMES=(["base_x","base_y","base_sin","base_cos","vb_x","vb_y","vb_wz"]+[f"q_{a}" for a in ARM]+[f"qd_{a}" for a in ARM]+["grip_w"]
            +[f"ee_{c}" for c in "xyz"]+[f"ee_xaxis_{c}" for c in "xyz"]+[f"ee_zaxis_{c}" for c in "xyz"]
            +[f"obj_{c}" for c in "xyz"]+[f"obj_minus_ee_{c}" for c in "xyz"]+["obj_sin4","obj_cos4","is_box","is_cyl","obj_upright"]
            +[f"goal_{c}" for c in "xyz"])
ACT_NAMES={"joint":["vb_x","vb_y","vb_wz"]+[f"dq_{a}" for a in ARM]+["grip"],
           "ee":["vb_x","vb_y","vb_wz","dee_x","dee_y","dee_z","eez_x","eez_y","eez_z","grip"]}

def Rz(yaw):
    c,s=np.cos(yaw),np.sin(yaw); return np.array([[c,-s,0],[s,c,0],[0,0,1.0]])
def quat2mat(q):
    R=np.zeros(9); mujoco.mju_quat2Mat(R,np.asarray(q,float)); return R.reshape(3,3)

def features(qpos,qvel,ee_pos,ee_quat,obj_pos,obj_quat,grip_w,kind):
    """한 시점 관측 → 특징 39 (모두 로봇 좌표: 원점 = 베이스 x · y, 회전 = 베이스 yaw, 높이는 world 그대로)"""
    x,y,yaw=qpos[0],qpos[1],qpos[2]; R=Rz(yaw); t=np.array([x,y,0.0])
    vb=np.r_[R[:2,:2].T@qvel[0:2],qvel[2]]
    Re=R.T@quat2mat(ee_quat); Ro=quat2mat(obj_quat); ee=R.T@(np.asarray(ee_pos)-t); ob=R.T@(np.asarray(obj_pos)-t)
    psi=np.arctan2(Ro[1,0],Ro[0,0])-yaw                         # 물체 yaw(로봇 기준)
    box=kind=="box"; s4,c4=(np.sin(4*psi),np.cos(4*psi)) if box else (0.0,0.0)   # 상자 = 90° 대칭 → 4ψ, 원기둥 = 축 대칭(0)
    goal=R.T@(GOAL_W-t)
    return np.r_[x,y,np.sin(yaw),np.cos(yaw),vb,qpos[3:7],qvel[3:7],grip_w,ee,Re[:,0],Re[:,2],ob,ob-ee,s4,c4,float(box),float(not box),Ro[2,2],goal].astype(np.float32)

class Kin:
    """기구학 전용 MjData(물리와 분리): 관절 목표 → 끝점 FK, 끝점 목표 → 팔 IK"""
    W_AX=0.3   # 축 오차 가중(m 대비) — 함태훈 action_format.py와 같은 값
    def __init__(s,m):
        s.m=m; s.d=mujoco.MjData(m); s.sid=m.site("left_ee").id
        jid=[m.joint(f"left_{a}_joint").id for a in ARM]; s.qa=np.array([m.jnt_qposadr[j] for j in jid]); s.da=np.array([m.jnt_dofadr[j] for j in jid])
        s.lo,s.hi=m.jnt_range[jid,0],m.jnt_range[jid,1]; s.jp=np.zeros((3,m.nv)); s.jr=np.zeros((3,m.nv))
    def fk(s,qfull,qarm):
        s.d.qpos[:]=qfull; s.d.qpos[s.qa]=qarm; mujoco.mj_kinematics(s.m,s.d); mujoco.mj_comPos(s.m,s.d)
        return s.d.site_xpos[s.sid].copy(),s.d.site_xmat[s.sid].reshape(3,3).copy()
    def ee_target_base(s,qfull,qarm):
        """관절 목표 qarm을 지금 베이스 pose에서 FK → (로봇 좌표 위치, 로봇 좌표 z축)"""
        p,R=s.fk(qfull,qarm); Rb=Rz(qfull[2]); t=np.array([qfull[0],qfull[1],0.0]); return Rb.T@(p-t),Rb.T@R[:,2]
    def ik(s,qfull,q0,pb,zb,iters=30,lam=0.01):
        """끝점 목표(로봇 좌표 위치 3 + z축 2) → 팔 4관절 (가중 DLS, 위치 3 + 축 3 = 6식 최소제곱, 직전 해 q0에서 시작)"""
        Rb=Rz(qfull[2]); t=np.array([qfull[0],qfull[1],0.0]); pw=t+Rb@pb; zw=Rb@(zb/max(np.linalg.norm(zb),1e-9)); q=np.clip(q0.copy(),s.lo,s.hi)
        for _ in range(iters):
            p,R=s.fk(qfull,q); z=R[:,2]; e=np.r_[pw-p,s.W_AX*(zw-z)]
            if np.linalg.norm(e[:3])<2e-4 and np.linalg.norm(zw-z)<2e-3: break
            mujoco.mj_jacSite(s.m,s.d,s.jp,s.jr,s.sid)
            Jz=np.cross(s.jr[:,s.da].T,z).T                      # dz/dq_k = ω_k × z
            J=np.vstack([s.jp[:,s.da],s.W_AX*Jz]); q=np.clip(q+J.T@np.linalg.solve(J@J.T+lam**2*np.eye(6),e),s.lo,s.hi)
        p,R=s.fk(qfull,q); return q,float(np.linalg.norm(pw-p))

def targets(out,qpos,ee_pos,action,kin):
    """한 시점 행동 → 학습 목표 벡터 (out = joint | ee)"""
    yaw=qpos[2]; R=Rz(yaw); vb=np.r_[R[:2,:2].T@action[0:2],action[2]]
    if out=="joint": return np.r_[vb,action[3:7]-qpos[3:7],action[7]].astype(np.float32)
    pb,zb=kin.ee_target_base(qpos,action[3:7]); t=np.array([qpos[0],qpos[1],0.0]); ee=R.T@(np.asarray(ee_pos)-t)
    return np.r_[vb,pb-ee,zb,action[7]].astype(np.float32)

def add_hist(X,Y,hist,dt):
    """선택 입력: hist = "" | "prev" | "time" | "prev,time"
       prev = 직전 주기 행동 목표(학습 = 시연의 Y[t−1], 롤아웃 = 정책 자신의 직전 출력), time = 에피소드 경과 시간 [s]"""
    cols=[X]
    if "prev" in hist: P_=np.vstack([Y[:1]*0+Y0(Y.shape[1]),Y[:-1]]); cols.append(P_)
    if "time" in hist: cols.append((np.arange(len(X))*dt)[:,None].astype(np.float32))
    return np.hstack(cols).astype(np.float32)

def Y0(n):
    """t = 0의 「직전 행동」: 정지 · 관절 목표 = 현재(잔차 0) · 그리퍼 벌림 (끝점 출력이면 z축 = 연직 위 가정 대신 0)"""
    y=np.zeros(n,np.float32); y[-1]=P.GRIPPER["stroke"]; return y

def chunk_targets(Q,A,K):
    """액션 청킹(ACT, Zhao 2023의 단순형): t 시점에서 앞으로 K주기 명령을 한 번에 → (T, K·8). 기준은 모두 t 시점 상태
       [베이스 R(yaw_t)ᵀ v, wz, 팔 목표(t+k) − 팔(t), 그리퍼(t+k)], 끝을 넘으면 마지막 명령 반복 (joint 출력만)"""
    T=len(A); out=np.zeros((T,K,8),np.float32)
    for t in range(T):
        R=Rz(Q[t,2])[:2,:2]; j=np.minimum(np.arange(t,t+K),T-1); a=A[j]
        out[t,:,0:2]=a[:,0:2]@R; out[t,:,2]=a[:,2]; out[t,:,3:7]=a[:,3:7]-Q[t,3:7]; out[t,:,7]=a[:,7]
    return out.reshape(T,K*8)

def load_dir(path,out,m=None,only_success=True,chunk=1):
    """HDF5 폴더 → 에피소드별 (X, Y, seed). 명세 v1.0 필드만 사용(spec 속성 확인). chunk > 1이면 Y = 앞으로 chunk주기 명령(joint)"""
    kin=Kin(m if m is not None else P.build()) if out=="ee" else None; eps=[]
    for f in sorted(glob.glob(os.path.join(path,"ep_*.hdf5"))):
        with h5py.File(f,"r") as h:
            assert h.attrs["spec"]=="b1-dataset-1.0",(f,h.attrs["spec"])
            if only_success and not bool(h.attrs["success"]): continue
            o=h["obs"]; Q=o["qpos"][:]; V=o["qvel"][:]; EP=o["ee_pos"][:]; EQ=o["ee_quat"][:]; OP=o["obj_pos"][:]; OQ=o["obj_quat"][:]; G=o["gripper_width"][:]
            A=h["action_label"][:] if "action_label" in h else h["action"][:]; kind=str(h.attrs["kind"]); seed=int(h.attrs["seed"])
        X=np.stack([features(Q[t],V[t],EP[t],EQ[t],OP[t],OQ[t],G[t],kind) for t in range(len(A))])
        Y=chunk_targets(Q,A,chunk) if chunk>1 else np.stack([targets(out,Q[t],EP[t],A[t],kin) for t in range(len(A))])
        eps.append((X,Y,seed))
    return eps

def live_features(m,d,I,kind):
    """롤아웃: 지금 MjData → 특징 (데이터 기록과 같은 순서 · 같은 함수)"""
    q=np.zeros(4); mujoco.mju_mat2Quat(q,d.site_xmat[I.ee]); ob=I.obj[kind]
    return features(d.qpos,d.qvel,d.site_xpos[I.ee],q,d.xpos[ob],d.xquat[ob],2*float(np.mean(d.qpos[I.fq])),kind)

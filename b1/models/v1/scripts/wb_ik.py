# 전신(베이스 3 + 팔 4 = 7자유도) 역기구학 모듈 (week07 커리큘럼 화) — 박종진
#   인터페이스 = docs/ik_interface.md §v2 (함태훈 w07 wb_jac.py 제안 시그니처 그대로 → 함태훈 제어기에서 import만 바꾸면 됨)
#     wb_jacobian(m, d, side, include_base=True, rot=False) -> (J[3×n] 또는 (Jp, Jr), dofs[n])   열 = base TX · TY · RZ, pitch · roll · yaw · elbow
#     wb_weights(include_base=True, w_xy=10, w_yaw=10)      -> W[n]   대각 가중치(클수록 덜 움직임), 팔 = 1
#     wdls(J, W, lam=0.03)                                  -> J#[n×m]  가중 DLS  J# = W⁻¹Jᵀ (J W⁻¹ Jᵀ + λ² I)⁻¹
#     limit_avoid(m, d, side, include_base=True, k=1.0)     -> q̇0[n]  영공간 2차 목표: 팔 관절 범위 중앙으로, 베이스 0
#     null_proj(J, W)                                       -> N[n×n]  정확한 가중 영공간 투영 I − J⁺_W J (λ 없음 → 1차 과제를 건드리지 않음)
#     solve_wb_ik(m, d, side, target_pos, target_axis=None, …) -> WBIKResult   반복 해법(위치 / 위치 + 그리퍼 축)
#     reach_of(m, d, side, grasp_off=0.045)                -> r [m]   어깨 pitch 관절점 → 파지 중심(ee − 45 mm, 그리퍼 축) 수평 거리 (week10)
#     reach_project(m, dd, side, x, r_max, inc, …)          -> x       r > r_max 이면 베이스 xy 를 목표 쪽으로 (r − r_max)만큼 옮김 (week10)
#     place_base_for_reach(m, d, side, target, r_max, …)   -> base0   베이스 사전 배치: 어깨 → 목표 수평 거리 ≤ r_max 가 되게 (week10)
#     load_reach_limit(params=None, case=None, payload=None) -> r_max 또는 None   config/params.yaml limits.reach_limit (auto = reach_limit_w10.json)
#     solve_wb_ik(…, reach_limit=None) — r_max [m] 를 주면 ① 베이스 사전 배치 후 ② 매 반복 r ≤ r_max 투영 ③ 끝에 r > r_max + 1 mm 이면 status = reach_limit
#   q̇ = J# ẋ + N q̇0,   q̇0 = 관절 한계 회피(팔) + 베이스 원위치 복귀(베이스 이동 최소)
#   MuJoCo base slide 축은 world 고정 → 베이스 열(TX, TY)은 world x · y 단위벡터, RZ 열 = ẑ × (p − p_RZ). 로봇 기준 속도 명령 = R(yaw)ᵀ [ẋ, ẏ]
import os,sys
from dataclasses import dataclass,asdict,field
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
BASE=["base_planar_joint_TX","base_planar_joint_TY","base_planar_joint_RZ"]
Z_BAND=(0.606,1.789)   # 팔 끝점 도달 높이(바닥 기준, workspace.md) — 베이스는 높이를 못 바꿈

def dofs(m,side,include_base=True):
    names=(BASE if include_base else [])+[f"{side}_{a}_joint" for a in ARM]
    return np.array([m.jnt_dofadr[m.joint(n).id] for n in names])

def qadr(m,side,include_base=True):
    names=(BASE if include_base else [])+[f"{side}_{a}_joint" for a in ARM]
    return np.array([m.jnt_qposadr[m.joint(n).id] for n in names])

def wb_jacobian(m,d,side,include_base=True,rot=False):
    """끝점 site 자코비안(world). rot=False → (Jp 3×n, dofs), rot=True → ((Jp, Jr), dofs). d는 mj_kinematics · mj_comPos 이후 상태"""
    jp=np.zeros((3,m.nv)); jr=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jp,jr,m.site(f"{side}_ee").id)
    dd=dofs(m,side,include_base); return ((jp[:,dd],jr[:,dd]) if rot else jp[:,dd]),dd

def wb_weights(include_base=True,w_xy=10.0,w_yaw=10.0):
    return np.array(([w_xy,w_xy,w_yaw] if include_base else [])+[1.0]*4,float)

def wdls(J,W,lam=0.03):
    Wi=np.diag(1.0/np.asarray(W,float))
    return Wi@J.T@np.linalg.inv(J@Wi@J.T+lam**2*np.eye(J.shape[0]))

def null_proj(J,W):
    Wi=np.diag(1.0/np.asarray(W,float)); Jp=Wi@J.T@np.linalg.pinv(J@Wi@J.T,rcond=1e-8)
    return np.eye(J.shape[1])-Jp@J

def limit_avoid(m,d,side,include_base=True,k=1.0):
    """H = ½ Σ ((q − q_mid)/(q_max − q_min))² 의 음의 기울기 → 팔을 범위 중앙 쪽으로 (함태훈 wb_jac.py와 같은 정의)"""
    jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]
    q=d.qpos[[m.jnt_qposadr[j] for j in jid]]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    g=-k*(q-(lo+hi)/2)/(hi-lo)**2
    return np.concatenate([np.zeros(3) if include_base else [],g])

@dataclass
class WBIKResult:
    q: np.ndarray            # (4,) 팔 [pitch, roll, yaw, elbow] rad
    base: np.ndarray         # (3,) 베이스 [x, y, yaw] (world, MuJoCo base_planar_joint TX · TY · RZ)
    success: bool
    pos_err: float           # [m]
    rot_err: float           # [rad] (축 미지정 0)
    iters: int
    status: str              # converged | orientation_limit | unreachable_height | base_limit | reach_limit | max_iter
    restarts_used: int = 0
    base_move: float = 0.0   # 시작 대비 베이스 평면 이동 [m]
    base_turn_deg: float = 0.0
    reach: float = float("nan")   # 해의 어깨 → 파지 중심 수평 거리 [m] (week10)
    def as_dict(self):
        r=asdict(self); r["q"]=[float(x) for x in self.q]; r["base"]=[float(x) for x in self.base]; return r

def _axis_err(zc,zt):
    e=np.cross(zc,zt); s=np.linalg.norm(e); ang=float(np.arctan2(s,zc@zt))
    return (e/s*ang if s>1e-12 else (np.zeros(3) if zc@zt>0 else np.array([1.0,0,0])*np.pi)),ang

FOOT=(0.288,0.236)   # 상판 반치수 x · y [m] (assumptions_b1.json base.plate_xy 0.576 × 0.472) — 베이스 외곽 근사
def _clip_base(b,box,yaw_range,table_x=None):
    if yaw_range is not None: b[2]=np.clip(b[2],*yaw_range)
    if table_x is not None:   # 테이블(x > table_x, world)을 뚫지 않게: 회전한 상판 사각형의 가장 앞 x ≤ table_x
        b[0]=min(b[0],table_x-(FOOT[0]*abs(np.cos(b[2]))+FOOT[1]*abs(np.sin(b[2]))))
    if box is not None:
        (xl,xh),(yl,yh)=box; b[0]=np.clip(b[0],xl,xh); b[1]=np.clip(b[1],yl,yh)
    return b

# ---- week10 도달 제한(자세 제한 방안) — 함태훈 r_max(sim/w10 reach_limit_w10.json)를 IK 제한값으로 ----
GRASP_OFF=0.045   # ee site(손바닥 아래면) → 파지 중심 [m] (pick_env.GRIPPER grasp_off · params.yaml gripper.grasp_off_m 와 같은 값)
def _shoulder_xy(m,d,side): return d.xanchor[m.joint(f"{side}_shoulder_pitch_joint").id][:2].copy()
def _grasp_xy(m,d,side,grasp_off=GRASP_OFF):
    sid=m.site(f"{side}_ee").id; R=d.site_xmat[sid].reshape(3,3); return (d.site_xpos[sid]-grasp_off*R[:,2])[:2].copy()
def reach_of(m,d,side,grasp_off=GRASP_OFF):
    """어깨 pitch 관절점 → 파지 중심 수평 거리 r [m] (함태훈 w10_common.Arm.terms 의 rho 와 같은 정의). d는 mj_kinematics 이후"""
    return float(np.linalg.norm(_grasp_xy(m,d,side,grasp_off)-_shoulder_xy(m,d,side)))
def reach_project(m,dd,side,x,r_max,inc=True,grasp_off=GRASP_OFF,box=None,yaw_range=None,table_x=None):
    """기준 상태 x(베이스 3 + 팔 4) 에서 r > r_max 이면 베이스 xy 를 파지 중심 쪽으로 (r − r_max) 평행이동(팔 자세 그대로 → r = r_max).
       베이스 한계(table_x · box)에 막히면 덜 옮겨지고 r > r_max 로 남음 → 호출 쪽에서 판정. 팔만(inc=False)이면 바꾸지 않음"""
    if r_max is None or not inc: return x
    QA=qadr(m,side,True); dd.qpos[QA]=x; mujoco.mj_kinematics(m,dd)
    s=_shoulder_xy(m,dd,side); g=_grasp_xy(m,dd,side,grasp_off); v=g-s; r=float(np.linalg.norm(v))
    if r<=r_max: return x
    x=x.copy(); x[:2]+=v/r*(r-r_max); x[:3]=_clip_base(x[:3],box,yaw_range,table_x); return x
def place_base_for_reach(m,d,side,target,r_max,base0,grasp_off=GRASP_OFF,box=None,yaw_range=None,table_x=None,frac=0.8):
    """베이스 사전 배치: 지금 베이스 yaw 그대로, 어깨 pitch 관절점 → 목표 수평 거리가 frac × r_max 이하가 되도록 베이스 xy 를 목표 쪽으로.
       (팔이 아직 목표에 없으므로 어깨 위치만 씀 — 파지 중심은 IK 후 r ≤ r_max 투영으로 맞춤)"""
    dd=mujoco.MjData(m); dd.qpos[:]=d.qpos; QA=qadr(m,side,True); dd.qpos[QA[:3]]=base0; mujoco.mj_kinematics(m,dd)
    v=np.asarray(target,float)[:2]-_shoulder_xy(m,dd,side); r=float(np.linalg.norm(v)); b=np.asarray(base0,float).copy()
    if r>frac*r_max: b[:2]+=v/r*(r-frac*r_max)
    return _clip_base(b,box,yaw_range,table_x)
def load_reach_limit(params=None,case=None,payload=None):
    """config/params.yaml limits.reach_limit → r_max [m] 또는 None(꺼짐 · 제한 불필요).
       r_max_m: auto → config/reach_limit_w10.json(함태훈 표)에서 (현재 case 어깨 pitch 구동기, 짐) 행. 짐이 표 사이면 더 무거운 쪽(보수적)."""
    import json,yaml
    root=os.path.dirname(os.path.dirname(os.path.dirname(V1)))     # 저장소 루트
    p=params if isinstance(params,dict) else yaml.safe_load(open(params or os.environ.get("B1_PARAMS",os.path.join(root,"config","params.yaml")),encoding="utf-8"))
    L=p["limits"]["reach_limit"]
    if not L["enable"]: return None
    if L["r_max_m"]!="auto": return float(L["r_max_m"])
    act=p["cases"][case or p["model"]["shoulder_case"]]["shoulder_pitch"]; mm=float(payload if payload is not None else p["payload"]["target_kg"])
    T=json.load(open(os.path.join(root,"config","reach_limit_w10.json"),encoding="utf-8"))
    rows=sorted([r for r in T["table"] if r["actuator"]==act],key=lambda r:r["payload"]); row=next((r for r in rows if r["payload"]>=mm-1e-9),None)
    if row is None: raise ValueError(f"reach_limit 표에 {act} {mm} kg 이상 행 없음 — 함태훈 w10_torque_decision 다시 실행")
    return None if row["full_reach"] else float(row["r_max"])

def _once(m,dd,side,inc,tp,zt,x0,b0,W,lam,rot_weight,k_limit,k_base,pos_tol,rot_tol,max_iter,step_max,base_step,box,yaw_range,table_x,r_max=None):
    QA=qadr(m,side,inc); jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    x=x0.copy(); sid=m.site(f"{side}_ee").id; na=len(x); hist=[]
    for it in range(1,max_iter+1):
        dd.qpos[QA]=x; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd)
        p=dd.site_xpos[sid]; ep=tp-p; pe=float(np.linalg.norm(ep)); re=0.0
        (Jp,Jr),_=wb_jacobian(m,dd,side,inc,rot=True)
        if zt is None: J,e=Jp,ep
        else:
            zc=dd.site_xmat[sid].reshape(3,3)[:,2]; er,re=_axis_err(zc,zt); P=np.eye(3)-np.outer(zc,zc)
            J=np.vstack([Jp,rot_weight*(P@Jr)]); e=np.concatenate([ep,rot_weight*(P@er)])
        hist.append((pe,re))
        if pe<=pos_tol and (zt is None or re<=rot_tol) and (r_max is None or reach_of(m,dd,side)<=r_max+1e-3): return x,pe,re,it,"converged",hist
        q0=limit_avoid(m,dd,side,inc,k_limit)
        if inc: q0[:3]=-k_base*(x[:3]-b0)   # 베이스 원위치 쪽으로(이동 최소)
        dx=wdls(J,W,lam)@e+null_proj(J,W)@q0
        sa=np.linalg.norm(dx[-4:])/step_max; sb=(np.linalg.norm(dx[:2])/base_step) if inc else 0.0
        sc=max(sa,sb,1.0); x=x+dx/sc
        x[-4:]=np.clip(x[-4:],lo,hi)
        if inc: x[:3]=_clip_base(x[:3],box,yaw_range,table_x)
        if r_max is not None: x=reach_project(m,dd,side,x,r_max,inc,box=box,yaw_range=yaw_range,table_x=table_x)
        if it>40 and abs(hist[-40][0]-pe)<1e-7 and (zt is None or abs(hist[-40][1]-re)<1e-6): break
    return x,pe,re,it,"max_iter",hist

def solve_wb_ik(m,d,side,target_pos,target_axis=None,*,include_base=True,q_init=None,base_init=None,
                w_xy=10.0,w_yaw=10.0,lam=0.05,rot_weight=0.3,k_limit=0.5,k_base=0.05,pos_tol=1e-3,rot_tol=1e-2,
                max_iter=300,step_max=0.3,base_step=0.1,base_box=None,yaw_range=None,table_x=None,restarts=4,seed=0,reach_limit=None):
    """target_pos (3,) world [m]. target_axis (3,) = 그리퍼 접근 방향(ee site −z가 향할 방향, world), None이면 위치만.
    base_box = ((x_lo, x_hi), (y_lo, y_hi)) world 베이스 원점 허용 범위(주행 공간), yaw_range = (lo, hi) rad,
    table_x = 정면 테이블 앞 가장자리 world x(테이블은 x > table_x) → 상판 사각형(FOOT)이 테이블을 뚫지 않게 베이스 x 제한.
    include_base=False → 팔만(6주차 ik.py와 같은 문제를 같은 반복식으로 — 비교용). d는 복사해서 계산(호출 쪽 상태 그대로).
    reach_limit = r_max [m](week10 도달 제한): 베이스 사전 배치(place_base_for_reach) → 반복마다 reach_project → 끝 판정. None = 제한 없음(9주차와 같은 결과)"""
    dd=mujoco.MjData(m); dd.qpos[:]=d.qpos; inc=include_base
    tp=np.asarray(target_pos,float); zt=None
    if target_axis is not None: zt=-np.asarray(target_axis,float); zt/=np.linalg.norm(zt)
    QA=qadr(m,side,True); cur=d.qpos[QA].copy()
    b0=cur[:3].copy() if base_init is None else np.asarray(base_init,float)
    qa0=cur[3:] if q_init is None else np.asarray(q_init,float)
    if not inc: dd.qpos[QA[:3]]=b0
    r_max=reach_limit
    if r_max is not None and inc: b0=place_base_for_reach(m,d,side,tp,r_max,b0,box=base_box,yaw_range=yaw_range,table_x=table_x)
    W=wb_weights(inc,w_xy,w_yaw); rng=np.random.default_rng(seed); best=None
    jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    for k in range(restarts+1):
        SEEDS=(-np.pi/2,np.pi/2,-np.pi/4,np.pi/4,np.pi)   # 재시작 베이스 yaw 시작값(옆 · 대각 · 뒤로 붙이기 같은 먼 해 — 국소해 탈출)
        qa=qa0 if (k==0 or (inc and k<=len(SEEDS))) else rng.uniform(lo,hi)
        x0=np.concatenate([b0,qa]) if inc else qa.copy()
        if inc and k>0:
            x0[2]=b0[2]+SEEDS[k-1] if k<=len(SEEDS) else rng.uniform(-np.pi,np.pi)
            if yaw_range is not None: x0[2]=np.clip(x0[2],*yaw_range)
        if inc: x0[:3]=_clip_base(x0[:3].copy(),base_box,yaw_range,table_x)
        r=_once(m,dd,side,inc,tp,zt,x0,(b0 if k==0 else x0[:3].copy()) if inc else b0,W,lam,rot_weight,k_limit,k_base,pos_tol,rot_tol,max_iter,step_max,base_step,base_box,yaw_range,table_x,r_max)
        score=r[1]+(0 if zt is None else 0.1*r[2])
        if best is None or score<best[0]: best=(score,r,k)
        if r[4]=="converged": break
    x,pe,re,it,st,_=best[1]
    if st!="converged" and r_max is not None and pe<=pos_tol and (zt is None or re<=rot_tol): st="reach_limit"   # 닿지만 r > r_max (베이스 한계)
    if st not in ("converged","reach_limit"):
        if pe<=pos_tol and zt is not None: st="orientation_limit"
        elif not (Z_BAND[0]-pos_tol<=tp[2]<=Z_BAND[1]+pos_tol): st="unreachable_height"
        elif inc and (base_box is not None or table_x is not None):
            bx=_clip_base(x[:3].copy(),base_box,yaw_range,table_x); bb=x[:3].copy(); bb[0]+=1e-3; bb2=_clip_base(bb.copy(),base_box,yaw_range,table_x)
            if base_box is not None or abs(bb2[0]-bb[0])>1e-9: st="base_limit"
    base=x[:3] if inc else b0.copy(); q=x[-4:]
    dd.qpos[QA[:3]]=base; dd.qpos[QA[3:]]=q; mujoco.mj_kinematics(m,dd); rr=reach_of(m,dd,side)
    b0=cur[:3].copy() if base_init is None else np.asarray(base_init,float)   # 이동량은 호출 쪽 시작 베이스 기준(사전 배치 이동 포함)
    return WBIKResult(reach=rr,q=q,base=base,success=st=="converged",pos_err=pe,rot_err=re,iters=it,status=st,restarts_used=best[2],
                      base_move=float(np.linalg.norm(base[:2]-b0[:2])),base_turn_deg=float(np.rad2deg(base[2]-b0[2])))

def apply(m,d,side,res):
    """해를 d.qpos에 넣고 forward (그대로 position ctrl · 베이스 목표로 사용)"""
    QA=qadr(m,side,True); d.qpos[QA[:3]]=res.base; d.qpos[QA[3:]]=res.q; mujoco.mj_forward(m,d)

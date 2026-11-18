# week09 박종진 — 롤아웃 평가: 전문가 vs BC(관절 출력 · 끝점 출력), 랜덤화 없음 vs 있음
#   평가 seed = 2000 ~ 2049 (학습 · 검증 seed 1000 ~ 1099와 겹치지 않음 = 처음 보는 초기 상태). 같은 seed = 같은 물체 · 위치 · 질량 · 랜덤 표본
#   공통 판정(세 정책 동일, 끝 상태): 물체가 목표 원(40 mm) 안 · 테이블 위(±10 mm) · 기울기 < 15° · 손가락 2개가 동시에 물고 있지 않음(놓았음)
#                                  · 실행 중 로봇(손가락 제외) ↔ 테이블 접촉 없음 · 수치 발산 없음
#   BC는 34 s(전문가 최장 28.6 s × 1.2) 안에 판정 조건을 0.5 s 유지하면 조기 종료
#   실행: python policies/eval_bc.py --policies expert,joint,ee,joint_dart,joint_dart_c25,joint_dart_c50 --conds nom,rand --n 50   (~30 분, 2 프로세스, 조건별 부분 재실행 가능)
#   출력: policies/results/eval.json · eval.md
import os,sys,json,time,argparse,copy,collections
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,HERE); sys.path.insert(0,os.path.join(ROOT,"scenarios"))
import bc_common as B, rand_dyn
import pick_env as P, expert as E

T_MAX=34.0; HOLD_OK=0.5; RISE_LIFT=0.03; ENS_M=0.1   # ENS_M = 청킹 시간 앙상블 가중 감쇠(주기당)

def judge_now(m,d,I,task):
    """지금 상태 판정 → (성공 여부, 물체 xy 오차 mm, 기울기°, 높이 차 mm, 물고 있음)"""
    ob=I.obj[task["kind"]]; p=d.xpos[ob]; R=d.xmat[ob].reshape(3,3); tilt=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1))))
    exy=float(np.linalg.norm(p[:2]-B.GOAL_W[:2])); dz=float(p[2]-(P.TABLE["top"]+P.obj_half_h(task["kind"])))
    held=len(P.contacts(m,d,I)["pad_obj"])==2
    return (exy<=P.GOAL["tol"] and abs(dz)<0.01 and tilt<15 and not held),exy*1e3,tilt,dz*1e3,held

def classify(ok,ev,exy,tilt,dz,held,rise_max):
    if ok: return None
    if ev: return ev
    if rise_max<RISE_LIFT: return "grasp_miss"
    if dz<-50: return "drop"
    if tilt>=15: return "tipped"
    if held and abs(dz)>=10: return "timeout_holding"
    if exy<=P.GOAL["tol"] and held: return "not_released"
    return "place_miss"

def _bad(d):
    W=mujoco.mjtWarning; return any(d.warning[w].number>0 for w in (W.mjWARN_BADQACC,W.mjWARN_BADQVEL,W.mjWARN_BADQPOS))
def _table_hit(m,d,I):
    return [g for g in P.contacts(m,d,I)["robot_table"] if not g.startswith("left_pad")]

def run(m0,pol,seed,rand,frames=None,frame_every=5,renderer=None):
    m=copy.deepcopy(m0); rp=rand_dyn.apply(m,seed) if rand else None
    d=mujoco.MjData(m); task=P.sample_task(seed); I=P.reset(m,d,task); ob=I.obj[task["kind"]]; z0=d.xpos[ob][2]
    rise_max=0.0; ev=None; t_ok=0.0; t_lift=None; n=0; ik_res=[]
    if pol=="expert":
        ex=E.Expert(m,d,task,I)
        while not ex.done and n*E.DT<40.0:
            ex.step(d)
            for _ in range(E.NSUB): mujoco.mj_step(m,d)
            rise_max=max(rise_max,d.xpos[ob][2]-z0); n+=1
            if t_lift is None and rise_max>=RISE_LIFT: t_lift=n*E.DT
            if frames is not None and n%frame_every==0: renderer.update_scene(d,"overview"); frames.append(renderer.render().copy())
        if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
        ev=ex.result["fail"] if ex.result["fail"] in ("table_collision","sim_unstable","ik_fail") else None   # 계획 단계 실패는 그 이름 그대로
        own=bool(ex.result["success"])
    else:
        kin=B.Kin(m); qa=kin.qa; qprev=d.qpos[qa].copy(); stroke=P.GRIPPER["stroke"]; yprev=B.Y0(len(pol.ym)); K=getattr(pol,"chunk",1)
        ens=collections.defaultdict(list)                      # 청킹: 주기 n → [(예측한 주기, 절대 명령 8)] — 시간 앙상블(ACT)
        while n*E.DT<T_MAX:
            x=B.live_features(m,d,I,task["kind"])
            if "prev" in pol.hist: x=np.r_[x,yprev]
            if "time" in pol.hist: x=np.r_[x,n*E.DT]
            y=pol.act(x.astype(np.float32)); yprev=y.astype(np.float32); yaw=d.qpos[2]; R=B.Rz(yaw)
            if K>1:   # 앞으로 K주기 명령(기준 = 지금 상태)을 절대 명령으로 바꿔 저장 → 지금 주기 명령 = 그 주기에 대한 예측들의 가중 평균, ACT식 w_i = exp(−m·i), i = 0이 가장 오래된 예측
                C=y.reshape(K,8)
                for k in range(K): ens[n+k].append(np.r_[R[:2,:2]@C[k,0:2],C[k,2],d.qpos[qa]+C[k,3:7],C[k,7]])
                P_=np.array(ens.pop(n)); w=np.exp(-ENS_M*np.arange(len(P_))); a_=(w[:,None]*P_).sum(0)/w.sum()
                y=np.r_[R[:2,:2].T@a_[0:2],a_[2],a_[3:7]-d.qpos[qa],a_[7]]
            vw=R[:2,:2]@y[0:2]; wz=y[2]
            if pol.out=="joint": arm=d.qpos[qa]+y[3:7]
            else:
                t=np.array([d.qpos[0],d.qpos[1],0.0]); ee_b=R.T@(d.site_xpos[I.ee]-t)
                arm,res=kin.ik(d.qpos.copy(),qprev,ee_b+y[3:6],y[6:9]); qprev=arm; ik_res.append(res)
            act=np.r_[np.clip(vw,-1.5,1.5),np.clip(wz,-1.5708,1.5708),np.clip(arm,kin.lo,kin.hi),np.clip(y[-1],0.0,stroke)]
            E.apply_action(m,d,act,I.act)
            for _ in range(E.NSUB): mujoco.mj_step(m,d)
            n+=1; rise_max=max(rise_max,d.xpos[ob][2]-z0)
            if t_lift is None and rise_max>=RISE_LIFT: t_lift=n*E.DT
            if frames is not None and n%frame_every==0: renderer.update_scene(d,"overview"); frames.append(renderer.render().copy())
            if _bad(d): ev="sim_unstable"; break
            if _table_hit(m,d,I): ev="table_collision"; break
            if d.xpos[ob][2]<P.TABLE["top"]-0.1: break                     # 바닥으로 떨어짐
            ok,*_=judge_now(m,d,I,task)
            t_ok=t_ok+E.DT if (ok and rise_max>=RISE_LIFT) else 0.0
            if t_ok>=HOLD_OK: break
        own=None
    ok,exy,tilt,dz,held=judge_now(m,d,I,task)
    ok=ok and ev is None and rise_max>=RISE_LIFT
    fail=classify(ok,ev,exy,tilt,dz,held,rise_max)
    return dict(seed=seed,kind=task["kind"],mass=round(task["mass"],3),success=bool(ok),fail=fail,expert_own_success=own,goal_err_mm=round(exy,1),
                tilt_deg=round(tilt,1),rise_max_mm=round(float(rise_max)*1e3,1),t_lift=None if t_lift is None else round(t_lift,2),t_end=round(n*E.DT,2),
                ik_res_mm_max=round(max(ik_res)*1e3,2) if ik_res else None,rand=rp)

def _job(args):
    pol_name,cond,seeds=args; m0=P.build(cam=True)
    if pol_name=="expert": pol="expert"
    else:
        import torch; torch.set_num_threads(1)
        from train_bc import Policy; pol=Policy(os.path.join(HERE,"ckpt",f"bc_{pol_name}.pt"))
    out=[]; t0=time.time()
    for s in seeds:
        r=run(m0,pol,s,cond=="rand"); r.update(policy=pol_name,cond=cond); out.append(r)
        print(pol_name,cond,s,r["success"],r["fail"],r["goal_err_mm"],flush=True)
    return out,time.time()-t0

def wilson(k,n,z=1.96):
    if n==0: return (0,0)
    p=k/n; c=(p+z*z/(2*n))/(1+z*z/n); h=z*np.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n); return (round(float(100*(c-h)),1),round(float(100*(c+h)),1))

def summarize(rows):
    S={}
    G=collections.defaultdict(list)
    for r in rows: G[(r["policy"],r["cond"])].append(r)
    for (p,c),g in G.items():
        k=sum(r["success"] for r in g); n=len(g)
        S[f"{p}|{c}"]=dict(policy=p,cond=c,n=n,success=k,rate=round(100*k/n,1),ci95=wilson(k,n),fails=dict(collections.Counter(r["fail"] for r in g if not r["success"])),
                          lifted=sum(r["rise_max_mm"]>=RISE_LIFT*1e3 for r in g),goal_err_mm_med_success=float(np.median([r["goal_err_mm"] for r in g if r["success"]])) if k else None,
                          t_end_mean_success=round(float(np.mean([r["t_end"] for r in g if r["success"]])),2) if k else None)
    return S

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--policies",default="expert,joint,ee"); ap.add_argument("--conds",default="nom,rand")
    ap.add_argument("--n",type=int,default=50); ap.add_argument("--start",type=int,default=2000); ap.add_argument("--procs",type=int,default=2)
    ap.add_argument("--out",default="eval")
    a=ap.parse_args(); seeds=list(range(a.start,a.start+a.n))
    jobs=[(p,c,seeds[i::2]) for p in a.policies.split(",") for c in a.conds.split(",") for i in range(2)]   # 조건마다 2조각 → 프로세스 균형
    import multiprocessing as mp; t0=time.time()
    with mp.get_context("spawn").Pool(a.procs) as pool: R=pool.map(_job,jobs)
    rows=sorted([r for o,_ in R for r in o],key=lambda r:(r["policy"],r["cond"],r["seed"]))
    f=os.path.join(HERE,"results",f"{a.out}.json"); old=json.load(open(f)) if os.path.exists(f) else dict(rows=[])
    keep=[r for r in old["rows"] if (r["policy"],r["cond"]) not in {(x["policy"],x["cond"]) for x in rows}]   # 다른 조건 결과는 보존(부분 재실행)
    allrows=sorted(keep+rows,key=lambda r:(r["policy"],r["cond"],r["seed"]))
    out=dict(seeds=[a.start,a.start+a.n-1],t_max=T_MAX,rand_ranges={k:v for k,v in rand_dyn.RANGES.items()},summary=summarize(allrows),rows=allrows,
             seconds=round(time.time()-t0,1),mujoco=mujoco.__version__)
    json.dump(out,open(f,"w"),indent=1,ensure_ascii=False)
    for k,v in out["summary"].items(): print(k,v)

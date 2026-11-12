# week08 박종진 — 시연 데이터 수집: 전문가 정책(상태기계)으로 에피소드 자동 생성 → 에피소드 1개 = HDF5 1개
#   형식 = datasets/dataset_spec_draft.md (B1 초안 v0.1, 함태훈 docs/dataset_spec.md 받으면 그 형식으로 맞춤)
#   저장 주기 = 제어 주기 50 Hz (관측은 행동 직전 상태, 행동 = 그 주기 동안 유지한 명령)
#   재생 검증: 같은 seed로 reset → 저장된 action만 넣어 다시 시뮬 → qpos 전 구간 최대 차 (MuJoCo 단일 스레드 = 결정적이면 0)
#   실행: python scenarios/collect.py --n 100 --start 1000 --out datasets/raw      (~3 분, 대용량 → .gitignore)
#         python scenarios/collect.py --replay datasets/raw                     (재생 검증 → scenarios/results/collect.{json,md})
#         python scenarios/collect.py --n 1 --start 1000 --cam --out datasets/samples  (커밋용 샘플: 1000 = 카메라 96×96 포함 ~7 MB)
#         python scenarios/collect.py --n 2 --start 1001 --out datasets/samples        (1001 · 1002 = 카메라 없음 ~0.5 MB)
import os,sys,json,time,argparse,hashlib,glob
import numpy as np,mujoco,h5py
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,HERE)
from expert import Expert,apply_action,STAGES,NSUB,DT
from pick_env import build,reset,sample_task,GRIPPER,CONTACT,TABLE,GOAL,OBJ,OPTION,SPAWN
SPEC="b1-w08-draft-0.1"
ACTION_NAMES=["base_vx","base_vy","base_wz","left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw","left_elbow","gripper_opening"]

def model_hash(m):
    """모델 지문: 컴파일된 모델 바이너리 sha256 앞 12자리(같은 모델이어야 재생이 같음)"""
    f="/tmp/_b1_pick.mjb"; mujoco.mj_saveModel(m,f,None); return hashlib.sha256(open(f,"rb").read()).hexdigest()[:12]

def record(m,seed,cam=None,max_t=40.0):
    d=mujoco.MjData(m); task=sample_task(seed); I=reset(m,d,task); ex=Expert(m,d,task,I)
    ob=I.obj[task["kind"]]; B=dict(qpos=[],qvel=[],ee_pos=[],ee_quat=[],obj_pos=[],obj_quat=[],grip=[],stage=[],action=[],ctrl=[],time=[],img=[])
    n=0
    while not ex.done and n*DT<max_t:
        q=np.zeros(4); mujoco.mju_mat2Quat(q,d.site_xmat[I.ee])
        B["qpos"].append(d.qpos.copy()); B["qvel"].append(d.qvel.copy()); B["ee_pos"].append(d.site_xpos[I.ee].copy()); B["ee_quat"].append(q)
        B["obj_pos"].append(d.xpos[ob].copy()); B["obj_quat"].append(d.xquat[ob].copy()); B["grip"].append(2*float(np.mean(d.qpos[I.fq])))
        B["time"].append(d.time); B["stage"].append(STAGES.index(ex.stage))
        if cam is not None: cam[0].update_scene(d,"overview"); B["img"].append(cam[0].render().copy())
        a=ex.step(d); B["action"].append(np.asarray(a,float)); B["ctrl"].append(d.ctrl.copy())
        for _ in range(NSUB): mujoco.mj_step(m,d)
        n+=1
    if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
    B["final_qpos"]=d.qpos.copy()
    return ex.result,B,task

def save(path,res,B,task,mh):
    with h5py.File(path,"w") as f:
        f.attrs.update(dict(spec=SPEC,seed=task["seed"],kind=task["kind"],mass=task["mass"],obj_xy=task["pos"],obj_yaw=task["yaw"],success=bool(res["success"]),
                            fail=str(res["fail"]),fail_stage=str(res["fail_stage"]),goal_err_mm=res["goal_err_mm"],t_total=res["t_total"],control_hz=round(1/DT),
                            sim_dt=DT/NSUB,model_hash=mh,action_names=json.dumps(ACTION_NAMES),stage_names=json.dumps(STAGES),
                            params=json.dumps(dict(gripper=GRIPPER,contact=CONTACT,table=TABLE,goal=GOAL,obj=OBJ,option=OPTION,spawn=SPAWN),default=float),
                            events=json.dumps(res["events"])))
        g=f.create_group("obs"); kw=dict(compression="gzip",compression_opts=4)
        g.create_dataset("qpos",data=np.array(B["qpos"]),**kw); g.create_dataset("qvel",data=np.array(B["qvel"],np.float32),**kw)
        for k in ("ee_pos","ee_quat","obj_pos","obj_quat"): g.create_dataset(k,data=np.array(B[k],np.float32),**kw)
        g.create_dataset("gripper_width",data=np.array(B["grip"],np.float32),**kw); g.create_dataset("stage",data=np.array(B["stage"],np.int8),**kw)
        if B["img"]: g.create_dataset("images/overview",data=np.array(B["img"],np.uint8),chunks=(1,)+B["img"][0].shape,**kw)
        f.create_dataset("action",data=np.array(B["action"]),**kw); f.create_dataset("ctrl",data=np.array(B["ctrl"]),**kw)
        f.create_dataset("time",data=np.array(B["time"]),**kw); f.create_dataset("final_qpos",data=B["final_qpos"])

def replay(m,path,mh0):
    """저장된 action만으로 재생 → 저장된 obs.qpos · final_qpos와 비교 (최대 절대 차)"""
    with h5py.File(path,"r") as f:
        seed=int(f.attrs["seed"]); A=f["action"][:]; Q=f["obs/qpos"][:]; QF=f["final_qpos"][:]; mh=f.attrs["model_hash"]; ok=bool(f.attrs["success"])
        ob=f["obs/obj_pos"][:]
    d=mujoco.MjData(m); task=sample_task(seed); I=reset(m,d,task); err=0.0
    for t in range(len(A)):
        err=max(err,float(np.max(np.abs(d.qpos-Q[t]))))
        apply_action(m,d,A[t],I.act)
        for _ in range(NSUB): mujoco.mj_step(m,d)
    err=max(err,float(np.max(np.abs(d.qpos-QF))))
    # 성공 판정도 재생 결과로 다시: 최종 물체 위치가 목표 원 안
    p=d.xpos[I.obj[task["kind"]]]; g=np.array([TABLE["edge_x"]+GOAL["depth"],GOAL["y"]]); ok2=bool(np.linalg.norm(p[:2]-g)<=GOAL["tol"])
    return dict(file=os.path.basename(path),seed=seed,steps=len(A),max_abs_qpos_diff=err,success_saved=ok,success_replayed_goal=ok2,model_hash_match=(mh==mh0))

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--n",type=int,default=100); ap.add_argument("--start",type=int,default=1000)
    ap.add_argument("--out",default=os.path.join(ROOT,"datasets","raw")); ap.add_argument("--cam",action="store_true"); ap.add_argument("--replay")
    a=ap.parse_args()
    if a.replay:
        m=build(cam=True); mh0=model_hash(m); t0=time.time(); R=[replay(m,p,mh0) for p in sorted(glob.glob(os.path.join(a.replay,"*.hdf5")))]
        res_f=os.path.join(HERE,"results","collect.json"); C=json.load(open(res_f)) if os.path.exists(res_f) else {}
        key="replay" if len(R)>=20 else "replay_samples"   # 샘플 재생은 따로 기록
        C[key]=dict(n=len(R),max_abs_qpos_diff=max(r["max_abs_qpos_diff"] for r in R),all_zero=all(r["max_abs_qpos_diff"]==0 for r in R),
                         success_consistent=sum(r["success_saved"]==r["success_replayed_goal"] for r in R),hash_match=sum(r["model_hash_match"] for r in R),
                         seconds=round(time.time()-t0,1),rows=R)
        json.dump(C,open(res_f,"w"),indent=1,ensure_ascii=False); print({k:v for k,v in C[key].items() if k!="rows"}); sys.exit()
    os.makedirs(a.out,exist_ok=True); m=build(cam=True); mh=model_hash(m)
    if a.cam:   # 렌더 전용 사본(그림자 · 안티앨리어싱 끔 → CPU 렌더 0.15 → 0.04 s/장). 물리 모델(지문)은 그대로
        import copy; mr=copy.deepcopy(m); mr.vis.quality.shadowsize=0; mr.vis.quality.offsamples=0; cam=(mujoco.Renderer(mr,96,96),)
    else: cam=None
    t0=time.time(); rows=[]
    for sd in range(a.start,a.start+a.n):
        res,B,task=record(m,sd,cam); p=os.path.join(a.out,f"ep_{sd:05d}.hdf5"); save(p,res,B,task,mh)
        rows.append(dict(seed=sd,kind=task["kind"],mass=round(task["mass"],3),success=res["success"],fail=res["fail"],steps=len(B["action"]),
                         t_total=res["t_total"],kb=round(os.path.getsize(p)/1024,1)))
        print(rows[-1],flush=True)
    if not a.cam and a.n>=20:   # 통계는 본 수집(100개)에서만 기록 — 샘플 몇 개로 덮어쓰지 않게
        import collections
        C=dict(n=len(rows),success=sum(r["success"] for r in rows),fails=dict(collections.Counter(r["fail"] for r in rows if not r["success"])),
               steps_total=sum(r["steps"] for r in rows),mb_total=round(sum(r["kb"] for r in rows)/1024,1),seconds=round(time.time()-t0,1),model_hash=mh,
               seeds=[a.start,a.start+a.n-1],rows=rows)
        os.makedirs(os.path.join(HERE,"results"),exist_ok=True); json.dump(C,open(os.path.join(HERE,"results","collect.json"),"w"),indent=1,ensure_ascii=False)
        print({k:v for k,v in C.items() if k!="rows"})

# week09 박종진 — 노이즈 주입 시연 수집 (DART, Laskey et al. 2017): 공변량 이동 완화
#   BC 1차 결과: 시연 상태 분포 밖으로 조금만 벗어나면(베이스가 경유점에 못 미침 등) 되돌아오는 예시가 없어 오차가 쌓임(compounding error)
#   → 전문가가 행동할 때 실제로 넣는 명령에 작은 상관 노이즈(OU)를 더해 「궤적에서 벗어난 상태」를 만들고, 라벨은 그 상태에서 전문가가 낸 깨끗한 명령
#     전문가 = 기준 궤적 + 베이스 위치 피드백(Kp 2.0)이라 벗어난 상태에서의 라벨이 곧 「되돌아오는 행동」이 된다 (DAgger처럼 전문가를 새 상태에 다시 묻는 효과, 상태기계 이력 문제 없음)
#   저장 형식 = b1-dataset-1.0 그대로 + 확장 필드 1개: `action_label`(T, 8) = 노이즈 없는 전문가 명령(학습 라벨). `action` = 실제로 넣은 명령(재생 기준 — 명세 §4 유지)
#   그리퍼 명령에는 노이즈를 넣지 않음(파지 실패로 에피소드가 끊기는 것만 늘어남). 실패 에피소드도 저장하되 학습은 성공만
#   실행: python policies/collect_dart.py --n 100 --start 3000      (~3 분/100개, datasets/raw_dart/ — 커밋 안 함)
import os,sys,json,time,argparse
import numpy as np,mujoco,h5py
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,os.path.join(ROOT,"scenarios"))
import collect as C, expert as E, pick_env as P

SIG=np.array([0.05,0.05,0.08, 0.01,0.01,0.01,0.01, 0.0])   # OU 정상상태 표준편차: 베이스 m/s · m/s · rad/s, 팔 rad, 그리퍼 0 (팔 0.03이면 끝점 ~15 mm → 정렬 허용 6 mm 초과로 6/6 실패)
FINE=("ALIGN","DESCEND","GRASP","LIFT","PLACE","RELEASE"); FINE_SCALE=0.3   # 정밀 단계는 노이즈 30 %
TAU=0.3                                                       # 상관 시간 [s]

def record_dart(m,seed,max_t=40.0):
    rng=np.random.default_rng(50_000+seed); a=np.exp(-E.DT/TAU); z=np.zeros(8)
    d=mujoco.MjData(m); task=P.sample_task(seed); I=P.reset(m,d,task); ex=E.Expert(m,d,task,I); ob=I.obj[task["kind"]]
    B=dict(qpos=[],qvel=[],ee_pos=[],ee_quat=[],obj_pos=[],obj_quat=[],grip=[],stage=[],action=[],label=[],ctrl=[],time=[],img=[]); n=0
    while not ex.done and n*E.DT<max_t:
        q=np.zeros(4); mujoco.mju_mat2Quat(q,d.site_xmat[I.ee])
        B["qpos"].append(d.qpos.copy()); B["qvel"].append(d.qvel.copy()); B["ee_pos"].append(d.site_xpos[I.ee].copy()); B["ee_quat"].append(q)
        B["obj_pos"].append(d.xpos[ob].copy()); B["obj_quat"].append(d.xquat[ob].copy()); B["grip"].append(2*float(np.mean(d.qpos[I.fq])))
        B["time"].append(d.time); B["stage"].append(E.STAGES.index(ex.stage))
        lab=np.asarray(ex.step(d),float)                     # 전문가가 이 상태에서 낸 명령(라벨) — step()이 d.ctrl도 씀
        z=a*z+np.sqrt(1-a*a)*rng.standard_normal(8)*SIG; act=lab+z*(FINE_SCALE if ex.stage in FINE else 1.0)
        E.apply_action(m,d,act,I.act)                         # 실제로는 노이즈 섞인 명령을 넣음
        B["action"].append(act); B["label"].append(lab); B["ctrl"].append(d.ctrl.copy())
        for _ in range(E.NSUB): mujoco.mj_step(m,d)
        n+=1
    if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
    B["final_qpos"]=d.qpos.copy(); return ex.result,B,task

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--n",type=int,default=100); ap.add_argument("--start",type=int,default=3000)
    ap.add_argument("--out",default=os.path.join(ROOT,"datasets","raw_dart")); a=ap.parse_args()
    os.makedirs(a.out,exist_ok=True); m=P.build(cam=True); mh=C.model_hash(m); rows=[]; t0=time.time()
    for sd in range(a.start,a.start+a.n):
        res,B,task=record_dart(m,sd); p=os.path.join(a.out,f"ep_{sd:05d}.hdf5")
        try: C.save(p,res,B,task,mh)
        except AssertionError as e:   # 수치 발산(sim_unstable) 에피소드는 MuJoCo가 시간을 리셋 → 명세 check 실패 → 저장 안 함(기록만)
            if os.path.exists(p): os.remove(p)
            rows.append(dict(seed=sd,kind=task["kind"],success=False,fail=str(res["fail"]),saved=False,check=str(e))); print(rows[-1],flush=True); continue
        with h5py.File(p,"a") as f:
            f.create_dataset("action_label",data=np.array(B["label"]),compression="gzip",compression_opts=4)
            f.attrs["dart"]=json.dumps(dict(sigma=SIG.tolist(),tau=TAU,fine_stages=FINE,fine_scale=FINE_SCALE,note="action = 실제 넣은 명령(노이즈 포함), action_label = 전문가 명령(학습 라벨)"))
        rows.append(dict(seed=sd,kind=task["kind"],mass=round(task["mass"],3),success=res["success"],fail=res["fail"],steps=len(B["action"])))
        print(rows[-1],flush=True)
    json.dump(dict(n=len(rows),success=sum(r["success"] for r in rows),seconds=round(time.time()-t0,1),sigma=SIG.tolist(),tau=TAU,rows=rows),
              open(os.path.join(HERE,"results",f"collect_dart_{a.start}.json"),"w"),indent=1,ensure_ascii=False)

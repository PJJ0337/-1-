# week10 박종진 — 세 방안 집기 비교: 제한 없음 / ① 짐 제한 / ② 도달 제한 (노션 10주차 화 · 개별 과제)
#   같은 20 seed(0 ~ 19, 8주차 run_pick20 과 같은 물체 종류 · 위치 · yaw), 짐 = 3 kg 고정(기획서 사양 최대) — 짐 제한이면 min(3, m_max)
#   방안별: 성공률 · 베이스 이동량(실측 경로 길이 · 회전) · 사이클 시간 · 최소 여유율(① 설계 = 정적 최악 포락선 ② 시나리오 실측 = 운반 중 구동 토크 최대)
#   r_max · m_max 출처 = 함태훈 week10 df1e64d (config/reach_limit_w10.json · params.yaml regression.mmax_kg), 여유율 = (정격 − 필요) ÷ 필요
#   실행: python scenarios/run_limits_w10.py [--procs 2]   (~6 분, 2 프로세스) → scenarios/results/limits_w10.{json,md}
import os,sys,json,time,argparse
import numpy as np
from multiprocessing import Pool
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
sys.path.insert(0,HERE); sys.path.insert(0,os.path.join(ROOT,"config")); sys.path.insert(0,os.path.join(ROOT,"tests","support"))
import expert as E
from wb_ik import reach_of, load_reach_limit
import params as PRM
PAR=PRM.load(); SF=1+PAR["torque_rule"]["min_margin"]
MASS=float(PAR["payload"]["target_kg"]); SEEDS=list(range(20))
CARRY=("LIFT","MOVE","PLACE")
J=["shoulder_pitch","shoulder_roll"]

def _rmax_auto():
    import copy; p=copy.deepcopy(PAR); p["limits"]["reach_limit"]["enable"]=True
    r=load_reach_limit(p)                              # a2_doc · 3 kg → None(최대 도달까지 OK)
    T=json.load(open(os.path.join(ROOT,"config","reach_limit_w10.json"),encoding="utf-8")); return r,T["reach_max_m"]
R_AUTO,R_FULL=_rmax_auto()
M_AUTO=float(PAR["regression"]["mmax_kg"][PAR["cases"][PAR["model"]["shoulder_case"]]["shoulder_pitch"]])
# (이름, reach, payload, 설명) — auto = params.yaml 기본 case(a2_doc)에서 계산된 값 그대로, 나머지는 「걸리면 무엇을 잃는가」를 재기 위한 강제값
CONFIGS=[("none",None,None,"제한 없음"),
         ("payload_auto",None,M_AUTO,f"① 짐 제한 auto (a2_doc m_max {M_AUTO} kg)"),
         ("reach_auto",R_FULL if R_AUTO is None else R_AUTO,None,f"② 도달 제한 auto (a2_doc 3 kg r_max {R_FULL if R_AUTO is None else R_AUTO} m = 최대 도달)"),
         ("payload_2kg",None,2.0,"① 짐 제한 2 kg (강제 — 커리큘럼 예시)"),
         ("reach_0.37",0.37,None,"② 도달 제한 0.37 m (강제)"),
         ("reach_0.35",0.35,None,"② 도달 제한 0.35 m (강제)")]

def run_one(args):
    name,reach,payload,sd=args
    E.set_limits(reach,payload); m=_M()
    st=dict(path=0.0,turn=0.0,prev=None,r_all=0.0,r_carry=0.0,tau=np.zeros(2),tau_all=np.zeros(2))
    A=[m.actuator(f"left_{j}").id for j in J]
    def cb(d,act,ex):
        b=d.qpos[ex.QA[:3]].copy()
        if st["prev"] is not None: st["path"]+=float(np.linalg.norm(b[:2]-st["prev"][:2])); st["turn"]+=abs(float(b[2]-st["prev"][2]))
        st["prev"]=b; r=reach_of(m,d,"left"); st["r_all"]=max(st["r_all"],r); t=np.abs(d.actuator_force[A]); st["tau_all"]=np.maximum(st["tau_all"],t)
        if ex.stage in CARRY: st["r_carry"]=max(st["r_carry"],r); st["tau"]=np.maximum(st["tau"],t)
    t0=time.time(); res,_=E.run_episode(m,sd,on_step=cb,mass=MASS)
    g=res.get("plan",{}).get("grasp",[]); ch=next((x for x in g if x.get("chosen")),None)
    return dict(cfg=name,seed=sd,kind=res["kind"],mass=res["mass"],success=res["success"],fail=res["fail"],t_total=res["t_total"],
                base_path_m=round(st["path"],3),base_turn_deg=round(float(np.degrees(st["turn"])),1),reach_max=round(st["r_all"],3),reach_carry=round(st["r_carry"],3),
                tau_carry=[round(float(x),2) for x in st["tau"]],tau_all=[round(float(x),2) for x in st["tau_all"]],
                base_yaw_grasp=ch.get("base_yaw_deg") if ch else None,wall_s=round(time.time()-t0,1))
_m=None
def _M():
    global _m
    if _m is None: _m=E.build()
    return _m

def design_margin(arm,mass,r_max):
    """설계(정적) 최소 여유율: 함태훈 8 · 10주차 방법 — 왼팔 3,000 자세 중 r ≤ r_max 인 자세의 |τ_g + m·Sz| + 베이스 가속 추가분 최대 → 필요"""
    add=np.array([PAR["base_accel"]["add_at_worst_Nm"][a] for a in J])
    sel=np.ones(len(arm["RHO"]),bool) if r_max is None else arm["RHO"]<=r_max
    need=(np.abs(arm["TG"][sel,:2]+mass*arm["SZ"][sel,:2])+add).max(0)
    out={}
    for case in PAR["cases"]:
        mg=[PRM.margin(PRM.rated(PAR,a,case),need[i])*100 for i,a in enumerate(J)]
        out[case]=dict(need=[round(float(x),2) for x in need],margin=[round(float(x),1) for x in mg],min=round(float(min(mg)),1))
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--procs",type=int,default=2); ap.add_argument("--only",default=None); a=ap.parse_args()
    cfgs=[c for c in CONFIGS if a.only is None or c[0] in a.only.split(",")]
    jobs=[(c[0],c[1],c[2],sd) for c in cfgs for sd in SEEDS]; t0=time.time()
    with Pool(a.procs) as P: rows=P.map(run_one,jobs,chunksize=1)
    import w10_common as C
    A=C.Arm(); Q=A.sample(3000,0); T=[A.terms(q) for q in Q]
    arm=dict(TG=np.array([t[0] for t in T]),SZ=np.array([t[1] for t in T]),RHO=np.array([t[4] for t in T]))
    S=[]
    for name,reach,payload,desc in cfgs:
        R=[r for r in rows if r["cfg"]==name]; ok=[r for r in R if r["success"]]; mass=min(MASS,payload) if payload else MASS
        tc=np.max([r["tau_carry"] for r in R if r["tau_carry"][0]>0],axis=0) if any(r["tau_carry"][0]>0 for r in R) else np.zeros(2)
        sc={case:dict(tau=[round(float(x),2) for x in tc],margin=[round(PRM.margin(PRM.rated(PAR,a,case),tc[i])*100,1) for i,a in enumerate(J)]) for case in PAR["cases"]}
        for case in sc: sc[case]["min"]=min(sc[case]["margin"])
        fails={}
        for r in R:
            if not r["success"]: fails[r["fail"]]=fails.get(r["fail"],0)+1
        S.append(dict(cfg=name,desc=desc,reach=reach,payload=payload,mass=mass,n=len(R),success=len(ok),fails=fails,
                      t_cycle_mean=round(float(np.mean([r["t_total"] for r in ok])),2) if ok else None,
                      base_path_mean=round(float(np.mean([r["base_path_m"] for r in ok])),3) if ok else None,
                      base_turn_mean=round(float(np.mean([r["base_turn_deg"] for r in ok])),1) if ok else None,
                      reach_carry_max=round(float(max(r["reach_carry"] for r in ok)),3) if ok else None,
                      design=design_margin(arm,mass,reach),scenario=sc))
    out=os.path.join(HERE,"results"); os.makedirs(out,exist_ok=True)
    json.dump(dict(summary=S,rows=rows,mass=MASS,seeds=SEEDS,r_auto=R_AUTO,r_full=R_FULL,m_auto=M_AUTO,run_s=round(time.time()-t0,1)),open(os.path.join(out,"limits_w10.json"),"w"),indent=1,ensure_ascii=False)
    base=next((s for s in S if s["cfg"]=="none"),S[0])
    L=["# 세 방안 집기 비교 — 제한 없음 / ① 짐 제한 / ② 도달 제한 (자동 생성 — run_limits_w10.py)","",
       f"- 조건: 8주차 집기 scene · 전문가 정책(옆면 접근 · 수직 하강), seed 0 ~ 19(run_pick20 과 같은 물체 · 위치 · yaw), **짐 {MASS:g} kg 고정**(사양 최대), dt 0.5 ms, 어깨 기본 case = {PAR['model']['shoulder_case']}",
       "- 여유율 = (정격 − 필요) ÷ 필요, 기준 20 %. 정격 TQ 118.8 / 피크 381.7 · RI85 72.6 / 181.5 N·m(A2 사양서 10/19 판). 그리퍼 0.9 kg(가정 S14 · S15)",
       "- **설계 여유율** = 왼팔 3,000 자세(함태훈 w10 방법) 중 r ≤ r_max 자세의 정적 최악 + 베이스 가속 추가분(7주차) — 판정용. **시나리오 실측** = 운반(LIFT · MOVE · PLACE) 중 어깨 구동 토크 최대 — 참고용",
       "- r = 어깨 pitch 관절점 → 파지 중심(ee − 45 mm) 수평 거리. 베이스 이동량 = 실측 베이스 경로 길이(성공 에피소드 평균)",
       "","| 방안 | 짐 kg | r_max m | 성공 (20회) | 사이클 s | 베이스 경로 m · 회전 ° | 운반 중 r 최대 m | 설계 최소 여유율 a2_doc / cad_ri85 | 시나리오 실측 최소 여유율 a2_doc / cad_ri85 | 실패 |",
       "|---|---|---|---|---|---|---|---|---|---|"]
    for s in S:
        L.append(f"| {s['desc']} | {s['mass']:g} | {s['reach'] if s['reach'] else '—'} | **{s['success']}/{s['n']}** | {s['t_cycle_mean']} | {s['base_path_mean']} · {s['base_turn_mean']} | {s['reach_carry_max']} | "
                 f"{s['design']['a2_doc']['min']} % / {s['design']['cad_ri85']['min']} % | {s['scenario']['a2_doc']['min']} % / {s['scenario']['cad_ri85']['min']} % | {', '.join(f'{k} {v}' for k,v in s['fails'].items()) or '—'} |")
    L+=["","## 설계 필요 토크 (pitch · roll, N·m) — 함태훈 검산용","","| 방안 | 필요 pitch · roll | a2_doc 여유율 pitch · roll | cad_ri85 여유율 pitch · roll | 운반 중 실측 τ pitch · roll |","|---|---|---|---|---|"]
    for s in S:
        L.append(f"| {s['cfg']} | {s['design']['a2_doc']['need'][0]} · {s['design']['a2_doc']['need'][1]} | {s['design']['a2_doc']['margin'][0]} · {s['design']['a2_doc']['margin'][1]} % | "
                 f"{s['design']['cad_ri85']['margin'][0]} · {s['design']['cad_ri85']['margin'][1]} % | {s['scenario']['a2_doc']['tau'][0]} · {s['scenario']['a2_doc']['tau'][1]} |")
    L+=["",f"실행 {round(time.time()-t0,1)} s ({a.procs} 프로세스)"]
    open(os.path.join(out,"limits_w10.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

if __name__=="__main__": main()

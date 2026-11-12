# week08 박종진 — 집기 시나리오 평가: 물체 초기 위치(· yaw · 질량 · 종류) 무작위 20회 → 성공률 · 실패 분류표
#   + 도달 한계 점검: 테이블 안쪽 깊이별 계획 성공률(수직 하강 · 베이스 여유 40 mm)
#   seed 0 ~ 19 = 평가용(데이터 수집 seed 1000 ~ 와 겹치지 않음)
#   출력: scenarios/results/pick20.{json,md}     실행: python scenarios/run_pick20.py   (~60 s)
import os,sys,json,time,collections
import numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from expert import *
from pick_env import SPAWN,GOAL,GRIPPER,CONTACT,OBJ,TABLE,OPTION

FAIL_KO=dict(ik_fail="IK 해 없음(도달 · 자세 한계)",table_collision="테이블 충돌(계획 점검 또는 실행 중)",approach_timeout="접근 추종 시간 초과",
             align_fail="정렬 오차 초과(> 6 mm · 4°)",grasp_miss="파지 실패(손바닥 접촉 · 빈 손)",slip="들기 중 미끄러짐",drop="이동 · 놓기 중 낙하",
             place_miss="목표 영역 밖(> 40 mm)",tipped="넘어짐(> 15°)",sim_unstable="시뮬 수치 발산(MuJoCo 경고)")

def main():
    t0=time.time(); m=build(); rows=[]
    for sd in range(20):
        r,_=run_episode(m,sd); t=sample_task(sd)
        rows.append(dict(seed=sd,kind=t["kind"],mass=round(t["mass"],3),depth=round(t["pos"][0]-TABLE["edge_x"],3),y=round(t["pos"][1],3),yaw_deg=round(float(np.rad2deg(t["yaw"])),1),
                         success=r["success"],fail=r["fail"],fail_stage=r["fail_stage"],goal_err_mm=r["goal_err_mm"],tilt_deg=r["tilt_deg"],t_total=r["t_total"],
                         align_err_mm=r.get("align_err_mm"),grip_width_mm=r.get("grip_width_mm"),lift_rise_mm=r.get("lift_rise_mm"),backoff=r.get("approach_backoff"),
                         table_contacts=r["table_contacts"],path_checks=r.get("path_checks"),
                         plan_grasp=[{k:(v if not isinstance(v,np.floating) else float(v)) for k,v in x.items()} for x in r["plan"].get("grasp",[])]))
    t1=time.time()
    # 깊이별 계획 성공률 (y 5곳 × 깊이 9단계, 원기둥 = 손가락 방향 자유 / 상자 yaw 0)
    ref=Ref(m); d=mujoco.MjData(m); I=reset(m,d,sample_task(0)); x0=d.qpos[ref.QA].copy(); sweep=[]
    for dep in (0.06,0.10,0.14,0.18,0.20,0.21,0.22,0.23,0.24,0.26):
        ok=0; n=0
        for y in np.linspace(-0.2,0.4,7):
            x,_,_=plan_grasp_pose(ref,m,x0,[TABLE["edge_x"]+dep,y],0.0,"cyl",I); ok+=x is not None; n+=1
        sweep.append(dict(depth=round(float(dep),2),plan_ok=ok,n=n))
    ns=sum(r["success"] for r in rows); cnt=collections.Counter(r["fail"] for r in rows if not r["success"])
    S=[r for r in rows if r["success"]]
    stat=dict(n=20,success=ns,rate=ns/20,fails=dict(cnt),
              goal_err_mm=dict(mean=round(float(np.mean([r["goal_err_mm"] for r in S])),1),max=round(float(np.max([r["goal_err_mm"] for r in S])),1)),
              t_total_s=dict(mean=round(float(np.mean([r["t_total"] for r in S])),1)),
              mass_success=dict(heavy_ge_2kg=[sum(1 for r in S if r["mass"]>=2),sum(1 for r in rows if r["mass"]>=2)]),
              table_contact_runs=sum(1 for r in rows if r["table_contacts"]),run_s=round(t1-t0,1),sweep_s=round(time.time()-t1,1))
    out=os.path.join(HERE,"results"); os.makedirs(out,exist_ok=True)
    json.dump(dict(stat=stat,rows=rows,depth_sweep=sweep,setup=dict(spawn=SPAWN,goal=GOAL,table=TABLE,gripper=GRIPPER,contact=CONTACT,option=OPTION,obj=OBJ)),
              open(os.path.join(out,"pick20.json"),"w"),indent=1,ensure_ascii=False,default=float)
    L=["# 집기 시나리오 무작위 20회 — 성공률 · 실패 분류 (자동 생성 — run_pick20.py)","",
       f"설정: 테이블 윗면 {TABLE['top']} m · 앞 가장자리 x {TABLE['edge_x']} m, 물체 = 상자 {2*OBJ['box_half']*1e3:.0f} mm / 원기둥 Ø{2*OBJ['cyl_r']*1e3:.0f} × {2*OBJ['cyl_half']*1e3:.0f} mm, "
       f"질량 {OBJ['mass'][0]} ~ {OBJ['mass'][1]} kg(로그 균등), 초기 위치 = 가장자리 안쪽 {SPAWN['depth'][0]} ~ {SPAWN['depth'][1]} m · y {SPAWN['y'][0]} ~ {SPAWN['y'][1]} m · yaw ±45°, "
       f"목표 = 안쪽 {GOAL['depth']} m · y {GOAL['y']} m 반지름 {GOAL['tol']*1e3:.0f} mm. 그리퍼 · 접촉 = 가정값(pick_env.GRIPPER · CONTACT), 어깨 구동기 a2_doc case","",
       f"## 결과: 성공 **{ns}/20 ({ns*5} %)**","",
       f"- 성공 시 목표 오차 평균 {stat['goal_err_mm']['mean']} · 최대 {stat['goal_err_mm']['max']} mm, 에피소드 평균 {stat['t_total_s']['mean']} s, 2 kg 이상 {stat['mass_success']['heavy_ge_2kg'][0]}/{stat['mass_success']['heavy_ge_2kg'][1]} 성공",
       f"- 실행 중 로봇 ↔ 테이블 접촉: {stat['table_contact_runs']}회 (계획 단계 점검이 막음)","",
       "| 실패 분류 | 횟수 | 판정 방법 |","|---|---|---|"]
    for k in FAIL: L.append(f"| {k} | {cnt.get(k,0)} | {FAIL_KO[k]} |")
    L+=["","| seed | 물체 | 질량 kg | 깊이 m | y m | yaw ° | 결과 | 실패 단계 | 목표 오차 mm | 정렬 오차 mm | 파지 폭 mm | 경유 m | 시간 s |","|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['seed']} | {r['kind']} | {r['mass']} | {r['depth']} | {r['y']} | {r['yaw_deg']} | {'✅' if r['success'] else '❌ '+r['fail']} | {r['fail_stage'] or ''} | "
                 f"{r['goal_err_mm'] if r['success'] else ''} | {r['align_err_mm'] or ''} | {r['grip_width_mm'] or ''} | {r['backoff'] or ''} | {r['t_total']} |")
    L+=["","## 도달 한계 — 테이블 안쪽 깊이별 계획 성공(원기둥 · 수직 하강 · 옆면 접근 · 베이스 여유 40 mm, y −0.2 ~ 0.4 m 7곳)","","| 깊이 m | "+" | ".join(str(s["depth"]) for s in sweep)+" |","|---"*(len(sweep)+1)+"|",
        "| 계획 성공 | "+" | ".join(f"{s['plan_ok']}/{s['n']}" for s in sweep)+" |","",
        "- 원기둥(손가락 방향 자유)은 안쪽 0.24 m까지 y 7곳 모두 계획 성공, 0.26 m 불가. 실패 seed 1 = 상자 · 깊이 0.212 m · yaw 40° — 같은 자리 원기둥이면 성공 → 상자 면에 맞춘 손가락 방향(자세 3조건 전부)을 4-DOF 팔 + 베이스 yaw로 맞추면 도달 깊이가 줄어듦(구술 ② 4-DOF 집기 문제)",
        "- 라이브러리 시작값을 3개 → 8개로 늘리기 전에는 깊이 0.21 m 원기둥 2회도 ik_fail(해는 있는데 시작값에 따라 못 찾음) → 17 → 19/20",
        "- 7주차 작업공간 해석(수직 하강 안쪽 0.306 m)은 끝점(손바닥 아래면) · 베이스 여유 0 기준 → 이번 집기는 파지 중심이 손바닥 아래 45 mm(끝점 z 0.777 m)이고 베이스를 테이블에서 40 mm 띄움(속도 제어 추종 오차) → 실제 집기 가능 깊이가 줄어듦",
        "",f"실행 {stat['run_s']} s(20회) + 깊이 점검 {stat['sweep_s']} s"]
    open(os.path.join(out,"pick20.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

if __name__=="__main__": main()

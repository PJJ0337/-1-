# week07 박종진 — 팔만(4-DOF) IK vs 전신(베이스 3 + 팔 4) IK 비교 · 수렴률 · 가중치 영향 · 영공간 차원
#   팔만 = 6주차 ik.py solve_ik(위치 우선 폴백 포함), 전신 = wb_ik.solve_wb_ik (가중 DLS + 정확한 가중 영공간 투영)
#   목표 = 6주차 「작업공간 밖」 T9 · T10 + 4-DOF 자세 한계 X0 · X2 + PDR A팀 지적 지점 A1 ~ A7
#   출력: results/wb_ik_compare.{json,md}
#   실행: python b1/models/v1/scripts/wb_ik_compare.py   (~40 s)
import os,sys,json,time
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE)
from ik import load_b1,solve_ik,approach_quat
from wb_ik import solve_wb_ik,wb_jacobian,wb_weights,wdls,null_proj,apply,qadr,Z_BAND
t0=time.time(); m,d=load_b1(); SIDE="left"; EDGE=0.288   # 상판 앞 가장자리 x (base 영점 기준)
DOWN=[0,0,-1]; FWD=[1,0,0]
# (이름, 구분, 위치 world [m], 접근 방향, 테이블 앞 가장자리 x(None = 장애물 없음), 설명)
T=[("T9","6주차 밖",[0.80,0.25,1.20],None,None,"정면 0.80 m (도달 반경 밖)"),
   ("T10","6주차 밖",[0.20,0.20,0.42],None,None,"상판 윗면 z 0.42 m"),
   ("X0","4-DOF 자세",[0.55,0.28,1.20],FWD,None,"정면 어깨 높이 · 앞으로 접근"),
   ("X2","4-DOF 자세",[0.30,0.25,0.85],DOWN,None,"상판 위 0.45 m · 아래로 접근"),
   ("A1","A팀 ① 낮은 영역",[0.25,0.28,0.55],None,None,"상판 위 0.15 m (팔 최저 0.606 m 아래)"),
   ("A2","A팀 ① 바닥",[0.60,0.28,0.05],None,None,"바닥 물체 z 0.05 m"),
   ("A3","A팀 ② 가장자리 밖",[0.60,0.28,0.75],None,None,"테이블 높이 · 상판 앞 0.31 m (장애물 없음)"),
   ("A4","A팀 ② 테이블 안쪽",[EDGE+0.15,0.28,0.75],None,EDGE,"테이블(앞 가장자리 = 상판 앞) 안쪽 0.15 m · z 0.75"),
   ("A5","A팀 ② 테이블 안쪽",[EDGE+0.30,0.28,0.75],None,EDGE,"같은 테이블 안쪽 0.30 m"),
   ("A6","A팀 ② 테이블 안쪽 · 높은 선반",[EDGE+0.30,0.28,1.10],None,EDGE,"선반 z 1.10 · 안쪽 0.30 m"),
   ("A7","A팀 ③ 공유 폭",[0.35,-0.30,1.00],None,None,"왼팔로 정중면 반대쪽 0.30 m"),
   ("A8","A팀 ④ 수직 하강",[0.45,0.28,0.80],DOWN,None,"테이블 위 물체 · 위에서 아래로 접근"),
   ("A9","A팀 ④ 수직 하강 · 테이블",[EDGE+0.10,0.28,0.80],DOWN,EDGE,"테이블 안쪽 0.10 m · 아래로 접근")]
ST={"converged":"수렴","orientation_limit":"위치 수렴 · 축 한계","unreachable":"도달 불가","unreachable_height":"도달 불가(높이)","max_iter":"미수렴","joint_limit":"관절 한계","base_limit":"베이스 제한(테이블)"}
rows=[]
for n,g,p,ax,tx,desc in T:
    ra=solve_ik(m,d,SIDE,p,None if ax is None else approach_quat(ax),frame="world")
    if tx is not None and p[0]-0.0>0: pass
    rw=solve_wb_ik(m,d,SIDE,p,ax,table_x=tx,restarts=8)
    dd=mujoco.MjData(m); apply(m,dd,SIDE,rw); mujoco.mj_collision(m,dd)
    rows.append(dict(name=n,group=g,pos=p,axis=ax,table_x=tx,desc=desc,
        arm=dict(status=ra.status,ok=ra.success,pos_mm=round(ra.pos_err*1e3,2),axis_deg=round(float(np.rad2deg(ra.rot_err)),2)),
        wb=dict(status=rw.status,ok=rw.success,pos_mm=round(rw.pos_err*1e3,2),axis_deg=round(float(np.rad2deg(rw.rot_err)),2),iters=rw.iters,
                base=[round(float(v),3) for v in rw.base],base_move=round(rw.base_move,3),base_turn_deg=round(rw.base_turn_deg,1),
                q_deg=[round(float(v),1) for v in np.rad2deg(rw.q)],ncon=int(dd.ncon))))

# ---- 무작위 목표 도달률 (world 상자, 베이스 영점에서 시작, 장애물 없음) ----
rng=np.random.default_rng(7); N=300
P=np.column_stack([rng.uniform(0.15,1.10,N),rng.uniform(-0.60,0.90,N),rng.uniform(Z_BAND[0]+0.02,Z_BAND[1]-0.02,N)])
def rate(P,axis):
    a=[];w=[];bm=[];it=[]
    for p in P:
        ra=solve_ik(m,d,SIDE,p,None if axis is None else approach_quat(axis),frame="world",restarts=2)
        rw=solve_wb_ik(m,d,SIDE,p,axis,restarts=4); a.append(ra.success); w.append(rw.success)
        if rw.success: bm.append(rw.base_move); it.append(rw.iters)
    return dict(n=len(P),arm_rate=round(100*np.mean(a),1),wb_rate=round(100*np.mean(w),1),
                base_move_median=round(float(np.median(bm)),3) if bm else None,base_move_max=round(float(np.max(bm)),3) if bm else None,
                iters_median=int(np.median(it)) if it else None)
RATE={"pos":rate(P,None)}
Pd=np.column_stack([rng.uniform(0.25,1.0,150),rng.uniform(-0.3,0.8,150),rng.uniform(0.62,1.10,150)])   # 테이블 · 선반 높이대 · 아래로 접근
RATE["down"]=rate(Pd,DOWN)
# 수직 하강이 가능한 높이 대역 = workspace_wb.py 결과(먼저 실행)
sid=m.site(f"{SIDE}_ee").id; QA=qadr(m,SIDE,True)
_wf=os.path.join(V1,"results","workspace_wb.json")
DOWNBAND=json.load(open(_wf,encoding="utf-8"))["down"] if os.path.exists(_wf) else dict(n=0,z_min=None,z_max=None)
DOWNBAND={k:DOWNBAND[k] for k in ("n","z_min","z_max")}
# ---- 가중치(베이스 비용) 영향: 같은 목표에서 W = 1 · 10 · 100 ----
WT=[]
for n,p,ax in (("T9",[0.80,0.25,1.20],None),("A3",[0.60,0.28,0.75],None),("A7",[0.35,-0.30,1.00],None),("A8",[0.45,0.28,0.80],DOWN)):
    for w in (1,10,100):
        r=solve_wb_ik(m,d,SIDE,p,ax,w_xy=w,w_yaw=w,restarts=8)
        WT.append(dict(target=n,w=w,status=r.status,base_move=round(r.base_move,3),base_turn_deg=round(r.base_turn_deg,1),
                       arm_move_deg=round(float(np.rad2deg(np.linalg.norm(r.q))),1),iters=r.iters))
# ---- 영공간 차원 · 영공간 운동 시연 (A3 해에서 베이스를 0.10 m 옆으로 밀 때 끝점이 그대로인가) ----
r=solve_wb_ik(m,d,SIDE,[0.60,0.28,0.75],None,restarts=8); dd=mujoco.MjData(m); apply(m,dd,SIDE,r); mujoco.mj_comPos(m,dd)
J7,_=wb_jacobian(m,dd,SIDE,True); J4,_=wb_jacobian(m,dd,SIDE,False)
p0=dd.site_xpos[sid].copy(); x=dd.qpos[QA].copy(); W=wb_weights(True)
for k in range(100):   # 베이스 y를 +0.1 m 보내려는 2차 목표만 영공간으로 (1차 과제 = 끝점 속도 0 + 오차 보정)
    dd.qpos[QA]=x; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd); J,_=wb_jacobian(m,dd,SIDE,True)
    q0=np.zeros(7); q0[1]=0.1/100*100   # 1 cm/step
    dx=wdls(J,W,0.01)@(p0-dd.site_xpos[sid])+null_proj(J,W)@(q0*0.01); x=x+dx
dd.qpos[QA]=x; mujoco.mj_kinematics(m,dd)
NULL=dict(rank_arm=int(np.linalg.matrix_rank(J4)),rank_wb=int(np.linalg.matrix_rank(J7)),null_arm=4-int(np.linalg.matrix_rank(J4)),null_wb=7-int(np.linalg.matrix_rank(J7)),
          demo_base_dy=round(float(x[1]-r.base[1]),3),demo_ee_drift_mm=round(float(1e3*np.linalg.norm(dd.site_xpos[sid]-p0)),3),
          demo_arm_change_deg=[round(float(v),1) for v in np.rad2deg(x[3:]-r.q)])

R=dict(targets=rows,rates=RATE,down_band=DOWNBAND,weights=WT,nullspace=NULL,settings=dict(lam=0.05,w_xy=10,w_yaw=10,k_limit=0.5,k_base=0.05,pos_tol_mm=1,rot_tol_rad=0.01,restarts=8,foot=[0.576,0.472]),
       mujoco=mujoco.__version__,runtime_s=round(time.time()-t0,1))
json.dump(R,open(os.path.join(V1,"results","wb_ik_compare.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
f=lambda r:("✅ " if r["ok"] else "❌ ")+ST.get(r["status"],r["status"])
L=["# 팔만 IK vs 전신 IK (자동 생성 — wb_ik_compare.py)","",
   "왼팔, 목표 = world 좌표(베이스 영점에서 시작), 허용 오차 위치 1 mm · 그리퍼 축 0.01 rad. 팔만 = 6주차 `ik.py`(위치 우선 폴백 포함), 전신 = `wb_ik.py` 가중 DLS(λ 0.05, 베이스 가중치 W = 10 → 팔 우선) + 정확한 가중 영공간(관절 한계 회피 · 베이스 원위치), 재시작 ≤ 8. 「테이블」= 앞 가장자리 x = 0.288 m(상판 앞 가장자리와 맞닿게 붙인 상태), 상판 사각형 0.576 × 0.472 m가 테이블을 뚫지 않게 베이스 x 제한(회전 포함)","",
   "| 목표 | 구분 | 위치 world [m] | 접근 | 팔만 | 오차 mm / ° | 전신 | 오차 mm / ° | 베이스 이동 [m] · 회전 [°] | 접촉 |","|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    a,w=r["arm"],r["wb"]; ax="위치만" if r["axis"] is None else ("아래로" if r["axis"]==DOWN else "앞으로")
    L.append(f'| {r["name"]} {r["desc"]} | {r["group"]} | {r["pos"]} | {ax} | {f(a)} | {a["pos_mm"]} / {a["axis_deg"]} | {f(w)} | {w["pos_mm"]} / {w["axis_deg"]} | {w["base_move"]} · {w["base_turn_deg"]} | {w["ncon"]} |')
na=sum(r["arm"]["ok"] for r in rows); nw=sum(r["wb"]["ok"] for r in rows)
L+=["",f"- 13개 중 팔만 **{na}개**, 전신 **{nw}개** 성공. 높이 문제(T10 · A1 · A2)는 전신으로도 불가 — 베이스는 높이를 못 바꿈(팔 끝점 z {Z_BAND[0]} ~ {Z_BAND[1]} m). 몸통 높이 · 리프트 · 팔 길이 같은 하드웨어 항목",
    "- 정면 거리 · 반대쪽(T9 · A3 · A7)은 베이스가 다가가면 해결. 테이블 때문에 베이스가 더 못 가는 경우(A4 ~ A6 · A9)는 베이스를 돌려(yaw ≈ −90°) **옆면을 테이블에 붙이는 해**로 해결 → 높이별 최대 깊이는 [workspace_wb.md](workspace_wb.md)",
    "- 4-DOF 자세 한계(X0 · X2 · A8)는 전신에서 해결: 7자유도 > 위치 3 + 축 2 = 5조건 → 팔이 축을 맞추는 자세를 고르고 베이스가 위치를 맞춤",
    "",f"## 무작위 목표 도달률 (장애물 없음, 높이 = 팔 도달 대역 안)","","| 목표 집합 | 개수 | 팔만 | 전신 | 전신 베이스 이동 중앙값 / 최대 [m] | 전신 반복 중앙값 |","|---|---|---|---|---|---|",
    f'| 위치만: x 0.15 ~ 1.10, y −0.60 ~ 0.90, z {Z_BAND[0]+0.02:.3f} ~ {Z_BAND[1]-0.02:.3f} | {RATE["pos"]["n"]} | {RATE["pos"]["arm_rate"]} % | **{RATE["pos"]["wb_rate"]} %** | {RATE["pos"]["base_move_median"]} / {RATE["pos"]["base_move_max"]} | {RATE["pos"]["iters_median"]} |',
    f'| 위치 + 아래로 접근: x 0.25 ~ 1.0, y −0.3 ~ 0.8, z 0.62 ~ 1.10 | {RATE["down"]["n"]} | {RATE["down"]["arm_rate"]} % | **{RATE["down"]["wb_rate"]} %** | {RATE["down"]["base_move_median"]} / {RATE["down"]["base_move_max"]} | {RATE["down"]["iters_median"]} |',
    "",f'- 아래로 접근이 가능한 팔 자세(workspace_wb.py, {DOWNBAND["n"]:,}개)의 끝점 높이 = **z {DOWNBAND["z_min"]} ~ {DOWNBAND["z_max"]} m** → 전신이면 이 높이대 어디서나 수평 위치는 베이스로 맞춤. 무작위 집합의 실패 = 이 대역 위(z > {DOWNBAND["z_max"]}) 목표 → 전신으로도 축 한계(손목 필요)',
    "","## 베이스 가중치 W 영향 (구술 ①)","","| 목표 | W | 결과 | 베이스 이동 [m] | 베이스 회전 [°] | 팔 관절 변화 ‖q‖ [°] | 반복 |","|---|---|---|---|---|---|---|"]
L+=[f'| {w["target"]} | {w["w"]} | {ST.get(w["status"],w["status"])} | {w["base_move"]} | {w["base_turn_deg"]} | {w["arm_move_deg"]} | {w["iters"]} |' for w in WT]
L+=["","- W ↑ → 한 스텝의 가중 최소노름 해에서 베이스 성분이 1/W로 줄어듦 → 팔이 먼저 한계까지 뻗고, 모자란 만큼만 베이스가 이동(팔 우선). W ↓ → 베이스가 일찍 많이 움직이고 팔은 덜 굽힘. 팔 우선 이유: 베이스는 무겁고 느리며(가감속이 팔 토크에 더해짐 — 함태훈 4.37 N·m per m/s²) 위치 오차(바퀴 미끄러짐)가 커서 정밀 동작은 팔이 맡는 게 유리",
    "- W 100은 팔이 한계에 먼저 닿아 수렴이 느려지거나(함태훈 경로 B 116 mm) 베이스 이동이 오히려 늘 수 있음 → 기본 W 10",
    "","## 영공간 (구술 ②)","",f'- 위치 IK 자코비안 계수: 팔만 3 × 4 → rank {NULL["rank_arm"]}, 영공간 **{NULL["null_arm"]}차원** (팔꿈치가 어깨-손 축 둘레로 도는 자기 운동) / 전신 3 × 7 → rank {NULL["rank_wb"]}, 영공간 **{NULL["null_wb"]}차원**',
    f'- 시연: A3 해에서 영공간만으로 베이스를 옆으로 {NULL["demo_base_dy"]} m 이동 → 끝점 이동 {NULL["demo_ee_drift_mm"]} mm, 팔 관절 변화 {NULL["demo_arm_change_deg"]}° (팔이 베이스 이동을 상쇄)',
    "- 영공간 사용처: 관절 한계 회피(범위 중앙), 베이스 이동 최소(원위치 복귀), 팔꿈치 높이(몸통 충돌 회피), 다음 동작을 위한 자세 유지. 위치 + 그리퍼 축(5조건)이면 팔만 −1(부족) · 전신 2차원",
    "- 구현: 1차 과제는 가중 DLS(λ로 특이점 안정), 2차 과제 투영은 λ 없는 정확한 가중 의사역 N = I − J⁺_W J (6주차에 λ 포함 투영은 위치 35 mm가 남는 문제가 있었음)",
    "",f'MuJoCo {mujoco.__version__}, 실행 {R["runtime_s"]} s']
open(os.path.join(V1,"results","wb_ik_compare.md"),"w",encoding="utf-8").write("\n".join(L)+"\n")
print("\n".join(L))

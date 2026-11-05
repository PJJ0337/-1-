# week07 박종진 — 토크 예산표 B1 통합본 (A팀 전달용) + 옆면 접근(전신 IK) 자세의 3 kg 정적 토크 점검
#   입력: results/workspace_max_torque.json(박종진 정적 최악, 6주차) · inputs/torque_budget_w07_taehoon.csv(함태훈 8fa9750 사본:
#         집기 · 뻗기 · 베이스 설계 가속 · 계단 명령) · inputs/a2_actuators.json(정격 · 피크)
#   추가 계산: 7주차 전신 IK가 쓰는 「옆면 붙이기」(yaw ≈ −90°) · 「위에서 아래로」 자세가 정적 최악보다 큰지 (3 kg, 테이블 높이 · 안쪽 0.05 ~ 0.40 m)
#   출력: results/torque_budget_b1.{json,md}     실행: python b1/models/v1/scripts/torque_budget_b1.py (~20 s)
import os,sys,csv,json
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE)
from v1_mjio import load_model
from wb_ik import solve_wb_ik,apply,qadr
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; S="left"; PAY=3.0; RATED52=52.0
m=load_model(os.path.join(V1,"mjcf","scene.xml"),strip_gravcomp=True); d=mujoco.MjData(m)
DA=[m.jnt_dofadr[m.joint(f"{S}_{a}_joint").id] for a in ARM]; sid=m.site(f"{S}_ee").id
def tau_static(m,d):
    mujoco.mj_forward(m,d); jp=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jp,None,sid)
    return np.abs(d.qfrc_bias[DA]+jp[:,DA].T@np.array([0,0,PAY*9.81]))
cases=[]
for z in (0.75,0.80,0.90,1.00):
    for dep in (0.05,0.15,0.25,0.35,0.40):
        for ax in (None,[0,0,-1]):
            mujoco.mj_resetData(m,d); r=solve_wb_ik(m,d,S,[0.288+dep,0.28,z],ax,table_x=0.288,restarts=8)
            if not r.success: continue
            apply(m,d,S,r); t=tau_static(m,d)
            cases.append(dict(z=z,depth=dep,approach="위치만" if ax is None else "아래로",yaw_deg=round(float(np.rad2deg(r.base[2])),0),
                              q_deg=[round(float(v),1) for v in np.rad2deg(r.q)],tau=[round(float(v),2) for v in t]))
side_max=[max(c["tau"][i] for c in cases) for i in range(4)]
W=json.load(open(os.path.join(V1,"results","workspace_max_torque.json"),encoding="utf-8"))["results"]["3kg"]
TH=list(csv.DictReader(open(os.path.join(V1,"inputs","torque_budget_w07_taehoon.csv"),encoding="utf-8")))
rows=[]
for r in TH:
    a=r["joint"]; i=ARM.index(a); need=max(float(r["need"]),side_max[i]); rated=float(r["rated"])
    rows.append(dict(case=r["case"],joint=a,act=r["actuator"],rated=rated,peak=float(r["peak"]),static=float(r["static"]),pick=float(r["pick"]),
                     reach=float(r["reach"]),base=float(r["base_design"]),side=side_max[i],need=round(need,2),
                     margin=round((rated/need-1)*100,1),ok12=rated>=1.2*need,pdr52=round((RATED52/need-1)*100,2),step=float(r["base_step"]),
                     step_margin=round((rated/float(r["base_step"])-1)*100,1)))
R=dict(side_approach=dict(n=len(cases),max_tau=side_max,cases=cases),rows=rows,
       a_team_condition="구동기 사이징(역동역학) 기준 3 kg, 관절 속도 3 rad/s 이하에서 필요 토크 42.9 N·m, 52 기준 여유 21 %. 현재 B1 서보 게인으로는 2.0 s 이상 동작에서 20 %, 제어 튜닝 후 갱신 (함태훈 · 박종진 10/29 합의)")
json.dump(R,open(os.path.join(V1,"results","torque_budget_b1.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
L=["# 토크 예산표 — B1 통합본 week07 (자동 생성 — torque_budget_b1.py)","",
   "3 kg 짐(끝점), 그리퍼 0.5 kg box 가정, 왼팔(오른팔 거울 동일). 판정 규칙(노션 0절): **정격 ≥ 필요 토크 × 1.2**, 필요 토크 = 아래 열 중 최대. 계단 명령 열은 참고(베이스 명령에 가속 제한을 넣어 피함)","",
   "| 출처 | 열 | 조건 |","|---|---|---|",
   "| 박종진 6주차 | 정적 | 도달 가능 자세 30,000 + 국소 최적화 중 최악 중력 토크 (`workspace_max_torque.json`) |",
   "| 함태훈 6주차 | 집기 · 뻗기 | 집기 시나리오(IK + 5차 궤적) · 뻗기 1.5 s 서보 시뮬 최대 |",
   "| 함태훈 7주차 | 베이스 설계 가속 | 정적 + 베이스 a 1 m/s² · α 1.57 rad/s² · ω 1.57 rad/s 결합 관성 추가분(방향 최악) |",
   f"| 박종진 7주차 | 옆면 · 하강 접근 | 전신 IK 해(테이블 안쪽 0.05 ~ 0.40 m · z 0.75 ~ 1.00 · 위치만 / 아래로) {len(cases)}자세의 정적 토크 최대 |",
   "","| case | 관절 | 구동기 정격 / 피크 | 정적 | 집기 | 뻗기 | 베이스 가속 | 옆면 · 하강 접근 | **필요** | 정격 여유 % · ×1.2 | PDR 52 여유 % | 베이스 계단 명령 (정격 여유 %) |","|---|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    L.append(f'| {r["case"]} | {r["joint"]} | {r["act"]} {r["rated"]:g} / {r["peak"]:g} | {r["static"]:.2f} | {r["pick"]:.2f} | {r["reach"]:.2f} | {r["base"]:.2f} | {r["side"]:.2f} | **{r["need"]:.2f}** | {r["margin"]} · {"OK" if r["ok12"] else "NG"} | {r["pdr52"]}{" 주의" if r["pdr52"]<20 else ""} | {r["step"]:.2f} ({r["step_margin"]}) |')
L+=["",f'- 옆면 · 하강 접근 자세(7주차 전신 IK, {len(cases)}개) 정적 최대 = pitch {side_max[0]:.2f} · roll {side_max[1]:.2f} · yaw {side_max[2]:.2f} · elbow {side_max[3]:.2f} N·m → 정적 최악(42.92 · 39.77)보다 작음 → **필요 토크는 그대로 베이스 가속 포함 값(pitch 44.12)**',
    "- 두 case(어깨 TQ · RI85) 모두 정격 × 1.2 통과. PDR 52 N·m 기준 여유는 베이스 가속을 넣으면 pitch 17.85 % → **20 % 미달(주의)** — 베이스 가속 제한 · 동작 시간 조건과 같이 전달",
    "- 베이스 속도 계단 명령(현재 kv로 a 3.8 m/s², α 18.9 rad/s²)이면 cad_ri85 roll 정격 초과(−10 %) → **베이스 명령 가속 제한 ≤ 1 m/s² · ≤ 1.57 rad/s² 필수**",
    "","## A팀 전달 문안 (초안 — 박종진 · 함태훈 합의 후 발송)","",
    "> [B1 → A팀] 7주차 토크 예산 · 작업공간 회신",
    "> 1. 구동기 사이징(역동역학) 기준 3 kg, 관절 속도 3 rad/s 이하에서 필요 토크 42.9 N·m(어깨 pitch), 52 N·m 기준 여유 21 %. 베이스 가감속(1 m/s² · 1.57 rad/s²)을 더하면 44.1 N·m · 여유 17.85 %. A2 정격 대비 여유는 어깨 TQ 169 % · RI85 65 %로 두 case 모두 통과",
    "> 2. 조건: 베이스 명령 가속 제한 ≤ 1 m/s² · ≤ 1.57 rad/s²(계단 명령이면 RI85 roll 정격 초과), 3 kg에서 팔 동작 2.0 s 이상(현재 서보 게인 기준, 3 kg에서 감쇠비 0.75 → 0.53 · 게인 스케줄 적용 시 오버슈트 5.2 %)",
    "> 3. 작업공간(PDR 지적): 베이스+팔 전신 IK로 정면 먼 곳 · 반대쪽 · 수직 하강(z 0.605 ~ 0.992 m)은 하드웨어 변경 없이 해결, 테이블 안쪽은 옆면 접근으로 0.149 → 0.464 m(테이블 높이). **상판 위 0.2 m 이하 · 바닥은 베이스로 불가** → 몸통 리프트 · 허리 · 어깨 높이 검토 요청. 공유 작업공간 폭(0.21 m)도 몸에 고정이라 그대로",
    "> 4. 남은 가정값: 그리퍼 0.5 kg box, 어깨 구동기 a2_doc(팀장 확정 대기), armature 로터만, 하체 가속 사양 — 수령 시 재계산",
    "",f'- 함태훈 원본: `inputs/torque_budget_w07_taehoon.csv` = taehoonham0118-cell/-1- 8fa9750 `sim/w07/results/torque_budget_w07.csv` 사본 (수정 없음)']
open(os.path.join(V1,"results","torque_budget_b1.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

# week07 박종진 — 토크 예산표 B1 통합본 (A팀 전달용) + 옆면 접근(전신 IK) 자세의 3 kg 정적 토크 점검
#   입력: results/workspace_max_torque.json(박종진 정적 최악, 6주차) · inputs/torque_budget_w07_taehoon.csv(함태훈 8fa9750 사본:
#         집기 · 뻗기 · 베이스 설계 가속 · 계단 명령) · inputs/a2_actuators.json(정격 · 피크)
#   추가 계산: 7주차 전신 IK가 쓰는 「옆면 붙이기」(yaw ≈ −90°) · 「위에서 아래로」 자세가 정적 최악보다 큰지 (3 kg, 테이블 높이 · 안쪽 0.05 ~ 0.40 m)
#   week09 갱신(11/12): 함태훈 week08 7f434ae 파지 중 예산(inputs/torque_budget_w08_taehoon.csv: 3 kg @ 파지 중심(ee − 45 mm) + 손가락 0.4 kg 정적 최악,
#         + 베이스 가속, 옆면 접근 집기 실행 최대) → 「8주차 필요」 = max(7주차 필요, 그 값들) 를 대표 필요 토크로
#   week09 정정(11/19, 팀장 8주차 평가표 회신 · 11/19 메시지: 용어 「요구 정격」): 판정은 「실제 정격 ≥ 요구 정격 = 필요 토크 × 1.2」 하나로. 「PDR 52 N·m 기준 여유 %」 표현 삭제
#         (52는 6주차 필요 토크 42.9 × 1.2에서 나온 요구치였을 뿐 구동기 사양이 아님), A팀 문안 7 · 8주차 합본으로 「짐 2 kg 제한 / 최악 자세 금지」 선택지 삭제
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
W8={(r["case"],r["joint"]):r for r in csv.DictReader(open(os.path.join(V1,"inputs","torque_budget_w08_taehoon.csv"),encoding="utf-8"))}
for r in rows:
    w=W8[(r["case"],r["joint"])]; st8,ba,sc=float(w["static_w08"]),float(w["base_add"]),float(w["scenario_3kg"])
    n8=round(max(r["need"],st8+ba,sc),2)
    r.update(static_w08=st8,base_add=ba,scenario_3kg=sc,need_w08=n8,margin_w08=round((r["rated"]/n8-1)*100,1),ok12_w08=r["rated"]>=1.2*n8,
             pdr52_w08=round((RATED52/n8-1)*100,2),pdr52_ok12_w08=RATED52>=1.2*n8,taehoon_need_w08=float(w["need_w08"]))
R=dict(side_approach=dict(n=len(cases),max_tau=side_max,cases=cases),rows=rows,
       a_team_condition="구동기 사이징(역동역학) 기준 3 kg, 관절 속도 3 rad/s 이하에서 필요 토크 42.9 N·m, 52 기준 여유 21 %. 현재 B1 서보 게인으로는 2.0 s 이상 동작에서 20 %, 제어 튜닝 후 갱신 (함태훈 · 박종진 10/29 합의)")
json.dump(R,open(os.path.join(V1,"results","torque_budget_b1.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
L=["# 토크 예산표 — B1 통합본 (week07 → week09 갱신: 8주차 파지 중 예산 반영, 자동 생성 — torque_budget_b1.py)","",
   "## 8주차 갱신 — 파지 중 (함태훈 week08 7f434ae `grasp_torque.md`, 대표 필요 토크)","",
   "가정: 3 kg를 **파지 중심(ee − 45 mm)**, 그리퍼 0.9 kg(손바닥 0.5 + 손가락 0.2 × 2, pick_env 임시값), 옆면 접근 집기 실행(dt 0.5 ms). 8주차 필요 = max(7주차 필요, 8주차 정적 최악 + 베이스 가속 추가분, 집기 실행 최대)","",
   "판정(팀장 7 · 8주차 회신): **실제 정격 ≥ 요구 정격 = 필요 토크 × 1.2** 하나로. 정격 여유 % = 실제 정격 / 필요 토크 − 1 (참고)","",
   "| case | 관절 | 실제 정격 | 7주차 필요 | 8주차 정적 | + 베이스 가속 | 집기 실행 최대 | **8주차 필요** | **요구 정격 (× 1.2)** | 판정 | 정격 여유 % |","|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    L.append(f'| {r["case"]} | {r["joint"]} | {r["act"]} {r["rated"]:g} | {r["need"]:.2f} | {r["static_w08"]:.2f} | {r["base_add"]:.2f} | {r["scenario_3kg"]:.2f} | **{r["need_w08"]:.2f}** | **{1.2*r["need_w08"]:.1f}** | {"OK" if r["ok12_w08"] else "NG"} | {r["margin_w08"]} |')
p8=next(r for r in rows if r["case"]=="a2_doc" and r["joint"]=="shoulder_pitch"); r8=next(r for r in rows if r["case"]=="a2_doc" and r["joint"]=="shoulder_roll")
q8=next(r for r in rows if r["case"]=="cad_ri85" and r["joint"]=="shoulder_pitch")
L+=["",f'- 필요 토크 pitch {p8["need_w08"]:.2f} · roll {r8["need_w08"]:.2f} N·m (7주차 44.12 · 41.57에서 증가 — 짐이 파지 중심으로 45 mm 내려가고 손가락 0.4 kg 추가). 함태훈 표와 일치 확인',
    f'- **요구 정격 = pitch {1.2*p8["need_w08"]:.1f} · roll {1.2*r8["need_w08"]:.1f} N·m** → 후보 실제 정격 어깨 TQ 118.8 · RI85 72.6 **둘 다 충족**(정격 여유 TQ {p8["margin_w08"]} % · RI85 {q8["margin_w08"]} %)',
    "- 정정(11/19, 팀장 회신): 8주차까지 쓰던 「PDR 52 N·m 기준 여유 %」는 삭제 — 52는 6주차 필요 토크(42.9) × 1.2에서 나온 요구치라 필요 토크가 48.7로 오르면 요구 정격도 58.4로 같이 갱신될 뿐, 구동기 사양이 아님. 정적 최악(팔 앞으로 수평 + 3 kg)이 지배, 실제 옆면 집기 실행 최대는 pitch 9.83 · roll 8.69 N·m",
    "","## 7주차 표 (week07 태그 기준, 참고)","",
   "3 kg 짐(끝점), 그리퍼 0.5 kg box 가정, 왼팔(오른팔 거울 동일). 판정 규칙(노션 0절): **정격 ≥ 필요 토크 × 1.2**, 필요 토크 = 아래 열 중 최대. 계단 명령 열은 참고(베이스 명령에 가속 제한을 넣어 피함)","",
   "| 출처 | 열 | 조건 |","|---|---|---|",
   "| 박종진 6주차 | 정적 | 도달 가능 자세 30,000 + 국소 최적화 중 최악 중력 토크 (`workspace_max_torque.json`) |",
   "| 함태훈 6주차 | 집기 · 뻗기 | 집기 시나리오(IK + 5차 궤적) · 뻗기 1.5 s 서보 시뮬 최대 |",
   "| 함태훈 7주차 | 베이스 설계 가속 | 정적 + 베이스 a 1 m/s² · α 1.57 rad/s² · ω 1.57 rad/s 결합 관성 추가분(방향 최악) |",
   f"| 박종진 7주차 | 옆면 · 하강 접근 | 전신 IK 해(테이블 안쪽 0.05 ~ 0.40 m · z 0.75 ~ 1.00 · 위치만 / 아래로) {len(cases)}자세의 정적 토크 최대 |",
   "","| case | 관절 | 구동기 정격 / 피크 | 정적 | 집기 | 뻗기 | 베이스 가속 | 옆면 · 하강 접근 | **필요** | 요구 정격 (× 1.2) · 판정 | 정격 여유 % | 베이스 계단 명령 (정격 여유 %) |","|---|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    L.append(f'| {r["case"]} | {r["joint"]} | {r["act"]} {r["rated"]:g} / {r["peak"]:g} | {r["static"]:.2f} | {r["pick"]:.2f} | {r["reach"]:.2f} | {r["base"]:.2f} | {r["side"]:.2f} | **{r["need"]:.2f}** | {1.2*r["need"]:.1f} · {"OK" if r["ok12"] else "NG"} | {r["margin"]} | {r["step"]:.2f} ({r["step_margin"]}) |')
L+=["",f'- 옆면 · 하강 접근 자세(7주차 전신 IK, {len(cases)}개) 정적 최대 = pitch {side_max[0]:.2f} · roll {side_max[1]:.2f} · yaw {side_max[2]:.2f} · elbow {side_max[3]:.2f} N·m → 정적 최악(42.92 · 39.77)보다 작음 → **필요 토크는 그대로 베이스 가속 포함 값(pitch 44.12)**',
    "- 두 case(어깨 TQ · RI85) 모두 실제 정격 ≥ 요구 정격(필요 × 1.2) 통과 — 베이스 가속 제한 · 동작 시간 조건과 같이 전달",
    "- 베이스 속도 계단 명령(현재 kv로 a 3.8 m/s², α 18.9 rad/s²)이면 cad_ri85 roll 정격 초과(−10 %) → **베이스 명령 가속 제한 ≤ 1 m/s² · ≤ 1.57 rad/s² 필수**",
    "","## A팀 전달 문안 — 7 · 8주차 합본 (11/19 정정: 팀장 8주차 평가표 회신 반영. 발송은 팀장)","",
    "- 정정 내용: 「PDR 52 N·m 기준 여유 %」 · 「52 N·m 사양이면 짐 2 kg 제한 / 최악 자세 금지 중 선택」 삭제 → 판정은 「실제 정격 ≥ 요구 정격(필요 토크 × 1.2)」 하나. 7주차 끝점 도달(여유 0)과 8주차 실제 집기 깊이를 구분해 적음",
    "",
    "> [B1 → A팀] 7 · 8주차 토크 예산 · 작업공간 회신",
    f"> 1. 필요 토크(3 kg를 그리퍼 파지 중심(손바닥 아래 45 mm)에, 그리퍼 0.9 kg 가정, 베이스 가감속 1 m/s² · 1.57 rad/s² 포함, 정적 최악 = 팔 앞으로 수평): 어깨 pitch {p8['need_w08']:.1f} · roll {r8['need_w08']:.1f} N·m → **요구 정격 {1.2*p8['need_w08']:.1f} · {1.2*r8['need_w08']:.1f} N·m**. 후보 정격 어깨 TQ 118.8 · RI85 72.6 N·m **둘 다 충족 → 짐 3 kg 그대로 통과**. 실제 옆면 접근 집기(3 kg) 실행 중 최대는 pitch 9.8 · roll 8.7 N·m",
    "> 2. 조건: 베이스 명령 가속 제한 ≤ 1 m/s² · ≤ 1.57 rad/s²(계단 명령이면 RI85 roll 정격 초과), 3 kg에서 팔 동작 2.0 s 이상(현재 서보 게인 기준). **하체(베이스) 가속 사양도 회신 요청**",
    "> 3. 작업공간(PDR 지적): 베이스 + 팔 전신 IK로 정면 먼 곳 · 반대쪽 · 수직 하강(끝점 z 0.605 ~ 0.992 m)은 하드웨어 변경 없이 해결. 테이블 안쪽 도달은 **끝점 기준(여유 0, 7주차)** 위치만 0.464 m · 수직 하강 0.306 m, **실제 집기 기준(테이블 여유 · 경로 충돌 점검 포함, 8주차)** 원기둥 0.24 m. 상판 위 0.2 m 이하 · 바닥은 베이스로 불가 → 몸통 리프트 · 허리 · 어깨 높이 검토 요청. 공유 작업공간 폭(0.21 m)도 몸에 고정이라 그대로",
    "> 4. 남은 가정값: 그리퍼 0.9 kg(손바닥 box 0.5 + 손가락 0.2 × 2) · 파지 중심 위치, 어깨 구동기 a2_doc(확정 전까지), armature 로터만 — 수령 시 재계산",
    "",f'- 함태훈 원본: `inputs/torque_budget_w07_taehoon.csv` = taehoonham0118-cell/-1- 8fa9750 `sim/w07/results/torque_budget_w07.csv` 사본 (수정 없음), `inputs/torque_budget_w08_taehoon.csv` = 7f434ae `sim/w08/results/torque_budget_w08.csv` 사본 (수정 없음)']
open(os.path.join(V1,"results","torque_budget_b1.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

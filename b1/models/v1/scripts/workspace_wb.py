# week07 박종진 — 작업공간 확장 정량화: 팔만(베이스 고정) vs 전신(베이스 x · y · yaw)
#   A팀 PDR 지적 ① 낮은 영역 · 바닥 ② 테이블 가장자리 밖 도달 ③ 공유 작업공간 폭 ④ 수직 하강 접근 을 숫자로
#   방법: 6주차 workspace_points.npz(왼팔 29,757점, base 기준) 를 베이스 yaw로 돌리고, 상판 사각형(0.576 × 0.472 m)이
#         테이블(앞 가장자리 x = E, 테이블은 x > E)을 뚫지 않는 한 최대한 붙였을 때 「테이블 안쪽 깊이」= 끝점 x − E
#         · 팔만 = yaw 0, 상판 앞면이 가장자리에 닿음(6주차 표와 같음)  · 전신 = yaw −180 ~ 180°(1°) 중 최대
#   수직 하강 가능 자세: 무작위 20만 자세를 「그리퍼 축 = 연직 아래」 2조건 위로 Gauss-Newton 투영 → 끝점 높이 · 깊이 대역
#   출력: results/workspace_wb.{json,md}, img/workspace_wb.png
#   실행: python b1/models/v1/scripts/workspace_wb.py   (~40 s)
import os,sys,json,time
import numpy as np,mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE)
from ik import load_b1
from wb_ik import qadr,FOOT
for _f in ("Malgun Gothic","NanumGothic","Noto Sans CJK KR","AppleGothic"):
    if any(_f in x.name for x in matplotlib.font_manager.fontManager.ttflist): plt.rcParams["font.family"]=_f; break; plt.rcParams["axes.unicode_minus"]=False
t0=time.time(); Z=np.load(os.path.join(V1,"results","workspace_points.npz")); P=Z["left_ee"]; PR=Z["right_ee"]
HX,HY=FOOT; PLATE_Z=0.4
YAW=np.deg2rad(np.arange(-180,180,1.0))
def depth_at(Pts,yaw):
    c,s=np.cos(yaw),np.sin(yaw); xr=c*Pts[:,0]-s*Pts[:,1]; front=HX*abs(c)+HY*abs(s); return xr-front   # 테이블 가장자리 기준 안쪽 깊이
BANDS=[(round(z,2),round(z+0.1,2)) for z in np.arange(0.6,1.8,0.1)]
rows=[]
for z0,z1 in BANDS:
    sel=(P[:,2]>=z0)&(P[:,2]<z1); Pts=P[sel]
    if not sel.any(): continue
    da=float(depth_at(Pts,0.0).max())
    best=max(((float(depth_at(Pts,y).max()),y) for y in YAW)); dw,yw=best
    # 같은 높이에서 테이블 따라(가장자리와 평행) 끝점이 닿는 폭: 깊이 ≥ 0.05 m 인 점의 y 범위(팔만) — 전신은 베이스 y 이동으로 무제한
    dd=depth_at(Pts,0.0); wide=Pts[dd>=0.05]
    rows.append(dict(z=[z0,z1],above_plate=[round(z0-PLATE_Z,2),round(z1-PLATE_Z,2)],n=int(sel.sum()),arm_depth=round(da,3),wb_depth=round(dw,3),
                     wb_yaw_deg=round(float(np.rad2deg(yw)),0),gain=round(dw-da,3),
                     arm_strip_y=[round(float(wide[:,1].min()),3),round(float(wide[:,1].max()),3)] if len(wide) else None))
# yaw 별 곡선 (테이블 높이 0.7 ~ 0.8)
selT=(P[:,2]>=0.7)&(P[:,2]<0.8); curveT=[float(depth_at(P[selT],y).max()) for y in YAW]

# ---- 수직 하강(그리퍼 축 = 아래) 가능한 자세의 끝점 대역 ----
m,d=load_b1(); SIDE="left"; QA=qadr(m,SIDE,True); sid=m.site(f"{SIDE}_ee").id
jid=[m.joint(f"{SIDE}_{a}_joint").id for a in ["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
DA=np.array([m.jnt_dofadr[j] for j in jid]); dd=mujoco.MjData(m); jr=np.zeros((3,m.nv)); zt=np.array([0,0,1.0])   # ee z = +ẑ ⇔ 접근(−z) 아래
rng=np.random.default_rng(3); DP=[]; DQ=[]
for q in rng.uniform(lo,hi,(20000,4)):
    for it in range(25):
        dd.qpos[:]=0; dd.qpos[QA[3:]]=q; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd)
        zc=dd.site_xmat[sid].reshape(3,3)[:,2]; e=np.cross(zc,zt)
        if np.linalg.norm(e)<1e-4 and zc@zt>0: break
        mujoco.mj_jacSite(m,dd,None,jr,sid); J=jr[:,DA]; P_=np.eye(3)-np.outer(zc,zc); Jr=P_@J
        q=np.clip(q+np.linalg.pinv(Jr,rcond=1e-6)@(P_@e),lo,hi)
    if np.linalg.norm(e)<1e-3 and zc@zt>0:
        mujoco.mj_collision(m,dd)
        if dd.ncon==0: DP.append(dd.site_xpos[sid].copy()); DQ.append(q.copy())
DP=np.array(DP)
dz=[]
for z0,z1 in BANDS:
    s=(DP[:,2]>=z0)&(DP[:,2]<z1) if len(DP) else np.zeros(0,bool)
    if s.any(): dz.append(dict(z=[z0,z1],n=int(s.sum()),arm_depth=round(float(depth_at(DP[s],0.0).max()),3),wb_depth=round(max(float(depth_at(DP[s],y).max()) for y in YAW),3)))
DOWN=dict(n=int(len(DP)),z_min=round(float(DP[:,2].min()),3),z_max=round(float(DP[:,2].max()),3),bands=dz,
          note="무작위 2만 자세 → 축 연직 투영(Gauss-Newton), 수렴 · 자기 충돌 없음만")

# ---- 공유 작업공간: 몸에 고정된 양이라 베이스와 무관 (한 팔로 반대쪽은 가능해짐) ----
SH=dict(note="양팔이 같은 점에 동시에 닿는 영역은 base_link에 고정 → 베이스 이동 · 회전으로 위치만 옮겨지고 폭(y −0.105 ~ 0.105 m) · 부피(0.069 m³)는 그대로",
        one_arm_cross="한 팔로 정중면 반대쪽: 팔만 0.095 m → 전신 제한 없음(베이스 옆 이동), wb_ik_compare A7")
R=dict(table_depth=rows,yaw_curve_table_height=dict(yaw_deg=[float(v) for v in np.rad2deg(YAW)],depth=curveT),down=DOWN,shared=SH,
       foot=[2*HX,2*HY],runtime_s=round(time.time()-t0,1))
json.dump(R,open(os.path.join(V1,"results","workspace_wb.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)

# ---- 그림 ----
fig,ax=plt.subplots(1,3,figsize=(15,4.6))
zc_=[np.mean(r["z"]) for r in rows]
ax[0].plot([r["arm_depth"] for r in rows],zc_,"o-",label="팔만 (yaw 0, 정면)"); ax[0].plot([r["wb_depth"] for r in rows],zc_,"s-",label="전신 (최적 yaw)")
if dz: ax[0].plot([r["wb_depth"] for r in dz],[np.mean(r["z"]) for r in dz],"v--",label="전신 + 아래로 접근")
ax[0].axvline(0,color="k",lw=0.8); ax[0].axhspan(0,0.606,color="r",alpha=0.08); ax[0].axhline(0.4,color="gray",ls=":")
ax[0].text(0.01,0.45,"팔 도달 불가 (z < 0.606 m)\n베이스로도 불가",color="r",fontsize=9)
ax[0].set_xlabel("테이블 앞 가장자리 안쪽 깊이 [m]"); ax[0].set_ylabel("끝점 높이 z (바닥 기준) [m]"); ax[0].set_ylim(0,1.85); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
ax[0].set_title("① · ② 높이별 테이블 안쪽 도달 깊이")
ax[1].plot(np.rad2deg(YAW),curveT); ax[1].set_xlabel("베이스 yaw [°] (0 = 정면이 테이블)"); ax[1].set_ylabel("안쪽 깊이 [m]"); ax[1].grid(alpha=0.3)
ax[1].set_title("테이블 높이(z 0.7 ~ 0.8 m) 깊이 vs 베이스 yaw")
c,s=np.cos(0),np.sin(0); a1=ax[2]
for yaw,col,lab in ((0.0,"C0","팔만 yaw 0"),(np.deg2rad(rows[1]["wb_yaw_deg"]),"C1",f'전신 yaw {rows[1]["wb_yaw_deg"]:.0f}°')):
    c,s=np.cos(yaw),np.sin(yaw); Pt=P[selT]; xr=c*Pt[:,0]-s*Pt[:,1]-(HX*abs(c)+HY*abs(s)); yr=s*Pt[:,0]+c*Pt[:,1]
    a1.scatter(yr,xr,s=2,alpha=0.35,color=col,label=lab)
    cor=np.array([[HX,HY],[HX,-HY],[-HX,-HY],[-HX,HY],[HX,HY]]); cx=c*cor[:,0]-s*cor[:,1]-(HX*abs(c)+HY*abs(s)); cy=s*cor[:,0]+c*cor[:,1]; a1.plot(cy,cx,color=col,lw=1)
a1.axhspan(0,0.6,color="saddlebrown",alpha=0.1); a1.axhline(0,color="saddlebrown"); a1.text(-0.55,0.02,"테이블",color="saddlebrown")
a1.set_aspect("equal"); a1.set_xlabel("테이블 가장자리 방향 [m]"); a1.set_ylabel("안쪽 깊이 [m]"); a1.legend(fontsize=8,loc="lower right"); a1.grid(alpha=0.3)
a1.set_title("위에서 본 z 0.7 ~ 0.8 m 끝점 (상판 외곽선)")
plt.tight_layout(); plt.savefig(os.path.join(V1,"img","workspace_wb.png"),dpi=130)

L=["# 작업공간 확장 정량화 — 팔만 vs 전신 (자동 생성 — workspace_wb.py)","",
   f"입력 = 6주차 `workspace_points.npz` 왼팔 {len(P):,}점(base 기준, 자기 충돌 제외). 상판 사각형 {2*HX:.3f} × {2*HY:.3f} m를 정면 테이블(앞 가장자리 = 0, 테이블은 안쪽 +)에 뚫지 않게 최대로 붙였을 때 끝점이 들어가는 깊이. 팔만 = 베이스 정면(yaw 0) 고정 · 전신 = yaw −180 ~ 180°(1° 간격) 중 최대(베이스 x · y는 붙이기 · 가장자리 따라 이동으로 자유). 오른팔은 거울 대칭으로 같음","",
   "## ② 테이블 안쪽 도달 깊이 (높이별)","","| 끝점 z [m] | 상판 위 [m] | 팔만 [m] | 전신 [m] (그때 yaw) | 증가 [m] | 팔만 깊이 ≥ 0.05 m인 가장자리 방향 폭 y [m] |","|---|---|---|---|---|---|"]
for r in rows: L.append(f'| {r["z"][0]} ~ {r["z"][1]} | {r["above_plate"][0]} ~ {r["above_plate"][1]} | {r["arm_depth"]} | **{r["wb_depth"]}** ({r["wb_yaw_deg"]:.0f}°) | {r["gain"]:+.3f} | {r["arm_strip_y"]} |')
rt=[r for r in rows if r["z"][0]==0.7][0]
L+=["",f'- **테이블 높이(z 0.7 ~ 0.8 m)**: 팔만 {rt["arm_depth"]} m → 전신 **{rt["wb_depth"]} m** (yaw {rt["wb_yaw_deg"]:.0f}° = 왼쪽 옆면을 테이블에 붙임, {rt["gain"]:+.3f} m). 모든 높이에서 최적 yaw = −90°: 정면 접근은 상판 앞 0.288 m가 어깨(x = 0) 앞을 막지만, 옆면은 상판 반폭 0.236 m인데 어깨가 y 0.170 m에 있고 roll로 옆 도달이 길어서',
    "- 같은 팔 · 같은 상판으로 높이 전 구간 **약 +0.31 m 더 깊이** (하드웨어 변경 없음). 전신 IK도 같은 해를 찾음(wb_ik_compare A4 · A5 · A6 · A9 = 베이스 yaw ≈ −90°)",
    "- 가로 방향(테이블 가장자리를 따라)은 전신이면 무제한(베이스가 따라 이동). 팔만은 표 오른쪽 열 y 범위 안 물체만",
    "- 주의: 옆 접근 자세는 roll이 커서(어깨 옆으로 벌림) 3 kg 최악 roll 토크 자세(39.77 N·m, roll 85.9°)에 가까움 → 함태훈 토크 예산 · 8주차 집기에서 확인. 테이블 윗면 · 팔 링크 충돌은 이 분석에서 미검사(끝점만) → 8주차 테이블 scene에서 확인",
    "","## ① 낮은 영역 · 바닥","",f"- 끝점 최저 z = 0.606 m(상판 위 0.206 m)는 **베이스 x · y · yaw와 무관**(평면 이동) → 상판 위 0.2 m 이하 · 바닥은 전신으로도 불가 (wb_ik_compare T10 · A1 · A2 = 도달 불가(높이)). 대책 = 몸통 리프트 · 허리 pitch · 어깨 높이 조정(A팀 · A1 검토 항목)",
    "","## ④ 수직 하강(위에서 아래로 접근) 가능 영역","",
    f'- 그리퍼 축이 연직 아래인 팔 자세 {DOWN["n"]:,}개(무작위 2만 → 축 투영, 충돌 없음): 끝점 높이 **z {DOWN["z_min"]} ~ {DOWN["z_max"]} m** (상판 위 {DOWN["z_min"]-PLATE_Z:.3f} ~ {DOWN["z_max"]-PLATE_Z:.3f} m)',
    "- 팔만이면 이 자세들의 끝점(2차원 곡면 위)에 물체가 있을 때만 수직 하강 가능 → 임의 물체는 불가(6주차 X2 · 축 1.84° 남음). 전신이면 이 높이대 안에서 **수평 위치는 베이스로 맞춤** → 높이만 맞으면 어디서나","",
    "| 끝점 z [m] | 자세 수 | 수직 하강 · 테이블 안쪽 깊이 팔만 [m] | 전신 [m] |","|---|---|---|---|"]
L+=[f'| {r["z"][0]} ~ {r["z"][1]} | {r["n"]} | {r["arm_depth"]} | {r["wb_depth"]} |' for r in dz]
L+=["","- 팔만 깊이 ≈ 0 = 정면 수직 하강 자세에서는 손이 상판 앞 가장자리 근처(몸 쪽)에 있음 → 테이블 위 물체를 위에서 집으려면 **옆면 접근(yaw −90°)으로 안쪽 약 0.3 m까지**. 더 안쪽은 측면 접근(축 수평)이나 손목 2축 → 8주차 집기 시나리오의 접근 방향 결정 근거",
    "","## ③ 공유 작업공간","",f'- {SH["note"]}',f'- {SH["one_arm_cross"]}',"",
    "## A팀 지적 ↔ 전신 제어로 해결 여부","","| A팀 지적 (PDR) | 팔만 | 전신 (하드웨어 변경 없음) | 남는 대책 |","|---|---|---|---|",
    "| ① 상판 위 0.2 m 이하 · 바닥 | 불가 | **불가** (높이 불변) | 리프트 · 허리 · 어깨 높이 |",
    f'| ② 테이블 가장자리 밖(안쪽) | 테이블 높이 {rt["arm_depth"]} m | **{rt["wb_depth"]} m** (옆면 접근 yaw −90°), 가로 방향 무제한 | 정면 접근으로 더 깊이: 어깨 전방 이동 · 테이블 아래로 들어가는 상판 |',
    "| ② 정면 먼 곳(장애물 없음) | 0.437 m(테이블 높이) | **해결** (베이스 접근) | — |",
    "| ③ 공유 작업공간 폭 0.21 m | 0.21 m | 폭 그대로 (몸 고정), 한 팔로 반대쪽은 해결 | roll 하한(몸통 폭) |",
    f'| ④ 수직 하강 접근 | 2차원 곡면 위 점만 | **z {DOWN["z_min"]} ~ {DOWN["z_max"]} m 대역 어디서나** (수평 위치는 베이스) | 그 위 높이 · 테이블 안쪽은 손목 2축 |',
    "",f'실행 {R["runtime_s"]} s · 그림 [../img/workspace_wb.png](../img/workspace_wb.png)']
open(os.path.join(V1,"results","workspace_wb.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

# week09 박종진 — BC 결과 그림 · 표: results/eval.json + results/train_*.json → img/bc_eval.png, results/eval.md
#   실행: python policies/plot_week09.py
import os,sys,json,glob
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__))
for _f in ("Malgun Gothic","NanumGothic","Noto Sans CJK KR","AppleGothic"):
    if any(_f in x.name for x in matplotlib.font_manager.fontManager.ttflist): plt.rcParams["font.family"]=_f; break
plt.rcParams["axes.unicode_minus"]=False
E=json.load(open(os.path.join(HERE,"results","eval.json"))); S=E["summary"]
LABEL={"expert":"전문가(상태기계)","joint":"BC 관절 · 시연 100","ee":"BC 끝점 · 시연 100","joint_dart":"BC 관절 · +DART","ee_dart":"BC 끝점 · +DART",
       "joint_dart_c10":"BC 관절 · +DART · 청킹 10","joint_dart_c25":"BC 관절 · +DART · 청킹 25","joint_dart_c50":"BC 관절 · +DART · 청킹 50"}
ORDER=[p for p in LABEL if f"{p}|nom" in S]
T={}
for f in glob.glob(os.path.join(HERE,"results","train_*.json")):
    j=json.load(open(f)); T[os.path.basename(f)[6:-5]]=j

fig,ax=plt.subplots(1,2,figsize=(12,4.2),gridspec_kw=dict(width_ratios=[1.6,1]))
x=np.arange(len(ORDER)); w=0.38
for i,(c,col) in enumerate((("nom","#4C72B0"),("rand","#DD8452"))):
    r=[S[f"{p}|{c}"]["rate"] if f"{p}|{c}" in S else np.nan for p in ORDER]
    lo=[S[f"{p}|{c}"]["ci95"][0] if f"{p}|{c}" in S else np.nan for p in ORDER]; hi=[S[f"{p}|{c}"]["ci95"][1] if f"{p}|{c}" in S else np.nan for p in ORDER]
    b=ax[0].bar(x+(i-0.5)*w,r,w,color=col,label=("랜덤화 없음" if c=="nom" else "랜덤화 있음(rand_dyn 7종)"),
                yerr=[np.array(r)-np.array(lo),np.array(hi)-np.array(r)],capsize=3,error_kw=dict(lw=0.8))
    for xi,ri in zip(x+(i-0.5)*w,r):
        if not np.isnan(ri): ax[0].text(xi,ri+2,f"{ri:.0f}",ha="center",fontsize=8)
ax[0].set_xticks(x); ax[0].set_xticklabels([LABEL[p] for p in ORDER],fontsize=8,rotation=15); ax[0].set_ylabel("성공률 % (평가 seed 2000 ~ 2049, 95 % CI)")
ax[0].set_ylim(0,110); ax[0].legend(fontsize=8,loc="upper right"); ax[0].grid(axis="y",alpha=0.3); ax[0].set_title("롤아웃 성공률 — 전문가 vs BC",fontsize=10)
for k in [p for p in ORDER if p in T]:
    h=T[k]["hist"]; ax[1].plot([e["epoch"] for e in h],[e["val"] for e in h],label=LABEL.get(k,k))
ax[1].set_yscale("log"); ax[1].set_xlabel("epoch"); ax[1].set_ylabel("검증 MSE (정규화, seed 1090 ~ 1099)"); ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3)
ax[1].set_title("학습 곡선 — 검증 손실은 낮아도 롤아웃은 실패(공변량 이동)",fontsize=9)
fig.tight_layout(); os.makedirs(os.path.join(HERE,"img"),exist_ok=True); fig.savefig(os.path.join(HERE,"img","bc_eval.png"),dpi=130)

FN={"grasp_miss":"못 집음","place_miss":"목표 밖에 놓음","table_collision":"테이블 충돌","timeout_holding":"들고 시간 초과","not_released":"안 놓음",
    "drop":"떨어뜨림","tipped":"넘어짐","sim_unstable":"수치 발산","ik_fail":"계획 실패(IK)"}
L=["# BC 롤아웃 평가 — 전문가 vs BC (자동 생성 — `plot_week09.py`, week09 박종진)","",
   f"평가 seed {E['seeds'][0]} ~ {E['seeds'][1]} (학습 1000 ~ 1099 · DART 3000 ~ 3299와 겹치지 않음), 같은 seed = 같은 물체 · 위치 · 질량 · 랜덤 표본. 공통 판정 = 목표 원 40 mm 안 · 테이블 위 ±10 mm · 기울기 < 15° · 놓음 · 테이블 충돌 없음, BC 시간 한도 {E['t_max']} s. MuJoCo {E['mujoco']}","",
   "| 정책 | 학습 데이터 · 설정 | 검증 MSE | 성공 (랜덤화 없음) | 95 % CI | 성공 (랜덤화 있음) | 들어 올린 수 (없음 / 있음) | 실패 분류 (없음) | 실패 분류 (있음) |","|---|---|---|---|---|---|---|---|---|"]
for p in ORDER:
    a,b=S.get(f"{p}|nom"),S.get(f"{p}|rand"); t=T.get(p)
    cfg="—" if p=="expert" else f"{t['cfg']['out']} · 프레임 {t['n_train']:,} · hist '{t['cfg'].get('hist','')}' · 청킹 {t['cfg'].get('chunk',1)}" if t else "?"
    vl="—" if not t else f"{t['hist'][-1]['val']:.4f}"
    fs=lambda s:" · ".join(f"{FN.get(k,k)} {v}" for k,v in sorted(s["fails"].items(),key=lambda kv:-kv[1])) if s and s["fails"] else "—"
    L.append(f"| {LABEL[p]} | {cfg} | {vl} | **{a['success']}/{a['n']} ({a['rate']:.0f} %)** | {a['ci95'][0]:.0f} ~ {a['ci95'][1]:.0f} | "
             +(f"**{b['success']}/{b['n']} ({b['rate']:.0f} %)**" if b else "—")+f" | {a['lifted']} / {b['lifted'] if b else '—'} | {fs(a)} | {fs(b)} |")
L+=["","랜덤화 범위(임시판 `rand_dyn.py`, 함태훈 randomize.py로 교체 예정):",""]+[f"- {k}: {v[0][0]} ~ {v[0][1]} — {v[1]}" for k,v in E["rand_ranges"].items()]
open(os.path.join(HERE,"results","eval.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

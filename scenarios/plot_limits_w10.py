# week10 박종진 — 세 방안 비교 그림: results/limits_w10.json → img/limits_w10.png
#   왼쪽: 설정별 성공(20회) · 운반 중 r 최대  /  오른쪽: 설계 최소 여유율(a2_doc · cad_ri85) vs 기준 20 %
import os,json,numpy as np,matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in ("Noto Sans CJK KR","NanumGothic","Malgun Gothic"):
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams["font.family"]=f; break
plt.rcParams["axes.unicode_minus"]=False
HERE=os.path.dirname(os.path.abspath(__file__)); J=json.load(open(os.path.join(HERE,"results","limits_w10.json"),encoding="utf-8")); S=J["summary"]
lab={"none":"제한 없음\n3 kg","payload_auto":"① 짐 auto\n(11.04 kg)","reach_auto":"② 도달 auto\n(0.729 m)","payload_2kg":"① 짐 2 kg\n(강제)","reach_0.37":"② 도달 0.37 m\n(강제)","reach_0.35":"② 도달 0.35 m\n(강제)"}
x=np.arange(len(S)); fig,ax=plt.subplots(1,2,figsize=(12,4.2))
b=ax[0].bar(x,[s["success"] for s in S],color=["#4C72B0" if "auto" in s["cfg"] or s["cfg"]=="none" else "#DD8452" for s in S])
for i,s in enumerate(S): ax[0].text(i,s["success"]+0.3,f"{s['success']}/20"+(f"\nr≤{s['reach_carry_max']}" if s["reach_carry_max"] else ""),ha="center",fontsize=8)
ax[0].set_xticks(x,[lab[s["cfg"]] for s in S],fontsize=8); ax[0].set_ylim(0,24); ax[0].set_ylabel("집기 성공 (seed 0 ~ 19, 짐 3 kg)"); ax[0].set_title("성공 · 운반 중 r 최대 [m] (파랑 = params auto, 주황 = 강제)",fontsize=10)
w=0.38
for k,(case,c) in enumerate((("a2_doc","#4C72B0"),("cad_ri85","#55A868"))):
    v=[s["design"][case]["min"] for s in S]; ax[1].bar(x+(k-0.5)*w,v,w,label=f"{case} ({'어깨 TQ' if case=='a2_doc' else '어깨 RI85'})",color=c)
    for i,vv in enumerate(v): ax[1].text(i+(k-0.5)*w,vv+3,f"{vv:.0f}",ha="center",fontsize=7)
ax[1].axhline(20,color="r",ls="--",lw=1); ax[1].text(len(S)-0.5,23,"기준 20 %",color="r",ha="right",fontsize=8)
ax[1].set_xticks(x,[lab[s["cfg"]] for s in S],fontsize=8); ax[1].set_ylabel("설계 최소 여유율 [%] = (정격 − 필요) ÷ 필요"); ax[1].legend(fontsize=8,loc="upper left")
ax[1].set_title("어깨 pitch · roll 중 최소 (정적 최악 + 베이스 가속, r ≤ r_max 자세)",fontsize=10)
fig.tight_layout(); os.makedirs(os.path.join(HERE,"img"),exist_ok=True); fig.savefig(os.path.join(HERE,"img","limits_w10.png"),dpi=150); print("saved")

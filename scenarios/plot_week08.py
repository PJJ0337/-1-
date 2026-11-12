# week08 박종진 — 보고서 · README용 그림: ① 집기 장면 4컷(GIF에서) ② 무작위 20회 결과 + 깊이별 계획 성공 ③ 그리퍼 파지 유지(마찰 원뿔 비교)
#   입력: results/pick20.json · gripper_slip.json, img/pick_seed6.gif    출력: img/pick_frames.png · pick20.png · gripper_slip.png
#   실행: python scenarios/plot_week08.py   (run_pick20 · gripper_slip · render_pick_gif 다음)
import os,json
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
HERE=os.path.dirname(os.path.abspath(__file__)); R=os.path.join(HERE,"results"); IMG=os.path.join(HERE,"img")
for f in ("Malgun Gothic","NanumGothic","Noto Sans CJK JP","Noto Sans CJK KR","AppleGothic"):
    if any(f==x.name for x in matplotlib.font_manager.fontManager.ttflist): plt.rcParams["font.family"]=f; break
plt.rcParams["axes.unicode_minus"]=False; plt.rcParams["font.size"]=9

# ① 4컷
g=Image.open(os.path.join(IMG,"pick_seed6.gif")); keys=[]
for i in (21,50,63,88):
    g.seek(min(i,g.n_frames-1)); keys.append(g.convert("RGB"))
W=Image.new("RGB",(440*4+30,330),"white")
for k,im in enumerate(keys): W.paste(im,(k*450,0))
W.save(os.path.join(IMG,"pick_frames.png"))

# ② 20회 + 깊이
P=json.load(open(os.path.join(R,"pick20.json"),encoding="utf-8")); rows=P["rows"]; sw=P["depth_sweep"]
fig,ax=plt.subplots(1,2,figsize=(10,3.7),gridspec_kw=dict(width_ratios=[1.25,1]))
for r in rows:
    c="tab:green" if r["success"] else "tab:red"; mk="s" if r["kind"]=="box" else "o"
    ax[0].scatter(r["y"],r["depth"],s=25+45*r["mass"],c=c,marker=mk,alpha=0.75,edgecolors="k",linewidths=0.5)
    if not r["success"]: ax[0].annotate(f"seed {r['seed']} {r['fail']}\n(상자 yaw {r['yaw_deg']:.0f}°)",(r["y"],r["depth"]),xytext=(-150,8),textcoords="offset points",fontsize=8,color="tab:red",arrowprops=dict(arrowstyle="->",color="tab:red",lw=0.8))
g_=P["setup"]["goal"]; ax[0].scatter([g_["y"]],[g_["depth"]],marker="*",s=220,c="gold",edgecolors="k",zorder=5)
ax[0].annotate("목표(반지름 40 mm)",(g_["y"],g_["depth"]),xytext=(8,6),textcoords="offset points",fontsize=8)
ax[0].axhline(0.24,ls="--",c="gray",lw=0.8); ax[0].text(0.36,0.243,"원기둥 계획 한계 0.24 m",fontsize=7.5,color="gray",ha="right")
ax[0].set_xlabel("물체 y (world) [m]"); ax[0].set_ylabel("테이블 가장자리 안쪽 깊이 [m]")
st=P["stat"]; ax[0].set_title(f"무작위 20회: 성공 {st['success']}/20 (□ 상자 ○ 원기둥, 크기 = 질량 0.2 ~ 3 kg)",fontsize=9.5)
ax[0].set_xlim(-0.4,0.42); ax[0].set_ylim(0.05,0.27); ax[0].grid(alpha=0.3)
x=[s["depth"] for s in sw]; y=[s["plan_ok"]/s["n"]*100 for s in sw]
ax[1].bar(range(len(x)),y,color=["tab:green" if v==100 else ("tab:orange" if v>0 else "tab:red") for v in y])
for k,s_ in enumerate(sw): ax[1].text(k,max(y[k],0)+2,f"{s_['plan_ok']}/{s_['n']}",ha="center",fontsize=7.5)
ax[1].set_xticks(range(len(x))); ax[1].set_xticklabels([f"{v:.2f}" for v in x],rotation=45)
ax[1].set_xlabel("테이블 안쪽 깊이 [m] (원기둥, y 7곳)"); ax[1].set_ylabel("계획 성공 [%]"); ax[1].set_ylim(0,110)
ax[1].set_title("도달 한계 — 수직 하강 · 옆면 접근 · 베이스 여유 40 mm",fontsize=9.5); ax[1].grid(axis="y",alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(IMG,"pick20.png"),dpi=160); plt.close(fig)

# ③ 그리퍼: 마찰 × 파지력, 질량별 — 채택(elliptic) vs 기본(pyramidal), 칸 = 들기 + 2 s 유지 미끄러짐 mm
G=json.load(open(os.path.join(R,"gripper_slip.json"),encoding="utf-8"))
MU=[0.3,0.6,1.0]; F=[10.0,20.0,40.0]; M=[0.2,1.0,3.0]
fig,ax=plt.subplots(1,2,figsize=(10,3.0))
for a,(rows_,ttl) in zip(ax,((G["rows"],"채택: elliptic · impratio 10"),(G["pyramidal"]["rows"],"MuJoCo 기본: pyramidal · impratio 1"))):
    Z=np.zeros((len(MU),len(F)*len(M))); A=np.empty(Z.shape,object)
    for r in rows_:
        if r["kind"]!="box" or r["condim"]!=G["contact"]["condim"]: continue
        i=MU.index(r["mu"]); j=M.index(r["mass"])*len(F)+F.index(r["F"]); v=min(r["slip_lift_mm"]+r["slip_hold_mm"],100)
        Z[i,j]=v; A[i,j]="낙하" if r["dropped"] else f"{v:.0f}"
    a.imshow(Z,cmap="RdYlGn_r",vmin=0,vmax=30,aspect="auto")
    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]): a.text(j,i,A[i,j],ha="center",va="center",fontsize=7)
    a.set_yticks(range(3)); a.set_yticklabels([f"μ {m}" for m in MU])
    a.set_xticks(range(9)); a.set_xticklabels([f"{f:.0f}N"+(f"\n질량 {M[j//3]} kg" if j%3==1 else "") for j,f in enumerate(F*3)],fontsize=7)
    for k in (2.5,5.5): a.axvline(k,c="k",lw=1)
    th=G["theory_match"] if rows_ is G["rows"] else G["pyramidal"]["theory_match"]
    a.set_title(f"{ttl} — 이론(F ≥ mg/2μ ⇔ 유지) 일치 {th[0]}/{th[1]}",fontsize=9)
fig.text(0.5,0.01,"칸 = 상자, 들기 + 2 s 유지 중 손가락 대비 미끄러짐 [mm] (5 mm 미만 = 유지)",ha="center",fontsize=8)
fig.tight_layout(rect=(0,0.04,1,1)); fig.savefig(os.path.join(IMG,"gripper_slip.png"),dpi=160); plt.close(fig)
print("saved")

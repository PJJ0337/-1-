# A1 전달용: CAD 조립 편차 D1(1.4°) · D2(7 mm) 측정 부품 · 기준 그림 1장 (CAD 좌표, mm)
# 사용: python a1_deviation_figure.py <arm.stp> <out.png>
import os,sys,numpy as np
STP,OUTP=sys.argv[1],sys.argv[2]; sys.argv=[sys.argv[0],STP,"/tmp/_a1"]
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_to_links.py"),encoding="utf-8").read().split("links={}")[0])
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if "CJK" in f: plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
def pts(shp,n=2500):
    V,F=tris_of(shp)
    if len(V)==0: return np.zeros((0,3))
    if len(V)>n: V=V[np.random.default_rng(0).choice(len(V),n,replace=False)]
    return V
G={}  # 그룹 이름 → 점
def add(g,shp): G.setdefault(g,[]).append(pts(shp))
for s,lk,pn,shp,top in leaves:
    if pn.startswith(("planet","sun","ring","JIS","ISO","GSRAU","key","MRA","Elast","Prirob","MB0","encoder plate","cover plate","CRB","planet shaft")): continue
    t=top or ""
    grp=("L" if s=="L" else "R")+":"+lk
    add(grp,shp)
for k in G: G[k]=np.vstack(G[k])
COL={"shoulder_pitch_link":"#2a7ab0","shoulder_roll_link":"#e08a1e","shoulder_yaw_link":"#3a9a4a","elbow_link":"#c0392b","torso_link":"#888888"}
fig,ax=plt.subplots(1,2,figsize=(16,8.2),gridspec_kw=dict(width_ratios=[1,1.35]))
# (a) D1: left 팔 옆면 (CAD z-y 평면, x 방향에서 봄)
a=ax[0]
for k,P in G.items():
    if k.startswith("L:") and k[2:]!="torso_link": a.scatter(P[:,2],P[:,1],s=0.4,c=COL[k[2:]],alpha=0.35)
J2=np.array([159.5,-20.0,41.2]); J3=np.array([159.5,-202.3,-2.2]); J4=np.array([159.6,-296.1,38.3])
a.plot([J2[2]-90,J2[2]+60],[J2[1],J2[1]],"k-",lw=2); a.text(J2[2]-60,J2[1]-45,"J2 축(RI85:2 출력 디스크 대칭축) = 기준",fontsize=9,bbox=dict(fc="white",ec="none",alpha=.85))
ax3=np.array([0.001,-1,-0.024]); L=np.array([-60,330])
a.plot(J3[2]+ax3[2]*L,J3[1]+ax3[1]*L,"-",color="#c0392b",lw=2,label="J3 축 실측 (RI85:3)")
a.plot([J3[2],J3[2]],[J3[1]+60,J3[1]-330],"--",color="k",lw=1.2,label="공칭: J2와 직교 (−y_CAD)")
tilt=np.degrees(np.arctan2(0.024,1)); 
a.annotate(f"J3 기울기 {tilt:.2f}° — J2–J3 사이각 {90-tilt:.2f}° (설계 90° 예상)\n축선 300 mm 끝에서 공칭 대비 {300*0.024:.1f} mm 벗어남",xy=(J3[2]+ax3[2]*300,J3[1]+ax3[1]*300),xytext=(-165,-585),fontsize=10.5,color="#c0392b",arrowprops=dict(arrowstyle="->",color="#c0392b"))
a.annotate("J4 축(RI85:4)도\n같은 1.37° · 양팔 동일\n→ C브래킷 ↔ 상완원통 ·\nRI85:3 체결부 추정",xy=(J4[2],J4[1]),xytext=(60,-380),fontsize=9.5,arrowprops=dict(arrowstyle="->"))
a.set_xlim(-170,175); a.set_ylim(-620,90); a.set_xlabel("z_CAD [mm]"); a.set_ylabel("y_CAD [mm]"); a.set_aspect("equal"); a.set_title("D1. left 팔 옆면 — J3 · J4 축 1.37° 기울어짐",fontsize=13); a.legend(loc="upper left",fontsize=8.5); a.grid(alpha=.3)
# (b) D2: 정면 (CAD x-y)
b=ax[1]
for k,P in G.items():
    b.scatter(P[:,0],P[:,1],s=0.3,c=COL[k.split(":")[1]],alpha=0.3)
xm=XC; b.axvline(xm,color="k",ls="-.",lw=1.2); b.text(xm+4,40,f"좌우 대칭면 x = {xm:.2f}\n(양쪽 J1 출력 디스크 중심의 중점)",fontsize=9,bbox=dict(fc="white",ec="none",alpha=.85))
LP={"J2":(159.5,-20.0),"J3":(159.5,-202.3),"J4":(159.6,-296.1)}; RP={"J2":(-395.7,-20.0),"J3":(-388.6,-201.8),"J4":(-388.5,-295.6)}
for k in LP:
    mx=2*xm-LP[k][0]; b.plot(mx,LP[k][1],"o",mfc="none",mec="k",ms=10); b.plot(*RP[k],"x",color="#c0392b",ms=10,mew=2)
    b.annotate(f"{k}: {RP[k][0]-mx:+.1f} mm",xy=RP[k],xytext=(RP[k][0]+25,RP[k][1]+8),fontsize=10,color="#c0392b",bbox=dict(fc="white",ec="none",alpha=.85))
b.plot([],[],"o",mfc="none",mec="k",label="left 축 위치의 거울 (기대 위치)"); b.plot([],[],"x",color="#c0392b",label="right 실측 (RI85:5 · :7 · :8 출력 디스크 중심)")
b.legend(loc="lower center",fontsize=9); b.set_xlabel("x_CAD [mm]"); b.set_ylabel("y_CAD [mm]"); b.set_aspect("equal"); b.grid(alpha=.3)
b.set_title("D2. 정면 — right 팔 상완 이하(상완원통 · 칼라 · 요크 · RI85:7 · :8)가 대칭면 쪽으로 7.2~7.4 mm",fontsize=12)
fig.suptitle("양팔로봇 최종버전.stp 조립 편차 — 의도된 설계인지 확인 요청 (B1 박종진 · 함태훈, 2026-10-14)\n측정: OCCT로 각 RI85 output-v1 디스크의 대칭축(관성 주축) · 중심 추출 → joint_origin_check_v0.csv",fontsize=13)
plt.tight_layout(); plt.savefig(OUTP,dpi=110); print("saved")

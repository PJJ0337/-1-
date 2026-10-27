# IK 데모 렌더 (week06) — results/ik_results.json의 해로 왼팔이 목표 T1 → … → T10을 차례로 도달 (관절공간 최소 저크 보간, 운동학 재생)
#   목표 = 초록 구(수렴) / 빨강 구(작업공간 밖, 최근접 해), ee = 주황 site
#   출력: img/ik_targets.gif, img/ik_targets.png (대표 4장)
# 사용: Windows → python ik_demo.py / 리눅스 헤드리스 → MUJOCO_GL=glfw xvfb-run -a python ik_demo.py (또는 MUJOCO_GL=osmesa)
import os,sys,json,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE); from v1_mjio import load_model
m=load_model(os.path.join(V1,"mjcf","scene.xml")); d=mujoco.MjData(m)
R=json.load(open(os.path.join(V1,"results","ik_results.json"),encoding="utf-8"))
T=[t for t in R["targets"] if t["target"].startswith("T")]
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; adr=[m.jnt_qposadr[m.joint(f"left_{a}_joint").id] for a in ARM]
W,H=560,480; r=mujoco.Renderer(m,H,W)
cam=mujoco.MjvCamera(); cam.lookat[:]=(0.15,0.25,1.05); cam.distance=2.0; cam.azimuth=130; cam.elevation=-14
opt=mujoco.MjvOption(); opt.sitegroup[3]=1
def frame(q,tgt,ok):
    d.qpos[:]=0; d.qpos[adr]=q; mujoco.mj_forward(m,d); r.update_scene(d,cam,opt); s=r.scene
    if tgt is not None and s.ngeom<s.maxgeom:
        mujoco.mjv_initGeom(s.geoms[s.ngeom],mujoco.mjtGeom.mjGEOM_SPHERE,np.array([0.025,0,0]),np.asarray(tgt,float),np.eye(3).ravel(),
                            np.array([0.11,0.69,0.48,0.9] if ok else [0.85,0.2,0.2,0.9],np.float32)); s.ngeom+=1
    return r.render()
s5=lambda u: 10*u**3-15*u**4+6*u**5   # 최소 저크 보간 형상
frames=[]; q=np.zeros(4); stills={}
for t in T:
    q1=np.array(t["q"]); ok=t["success"]
    for k in range(1,25): frames.append(frame(q+(q1-q)*s5(k/24),t["pos"],ok))
    img=frame(q1,t["pos"],ok)
    for _ in range(10): frames.append(img)
    stills[t["target"]]=img; q=q1
for k in range(1,25): frames.append(frame(q*(1-s5(k/24)),None,True))
import imageio.v2 as iio
from PIL import Image,ImageDraw,ImageFont
fp=[f for f in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc","C:/Windows/Fonts/malgun.ttf") if os.path.exists(f)]
font=ImageFont.truetype(fp[0],18) if fp else ImageFont.load_default()
def label(img,txt):
    im=Image.fromarray(img); dr=ImageDraw.Draw(im); dr.rectangle((0,0,W,30),fill=(255,255,255)); dr.text((8,4),txt,fill=(11,11,11),font=font); return np.array(im)
out=[]; i=0
for t in T:
    for _ in range(34): out.append(label(frames[i],f'{t["target"]} — {"수렴" if t["success"] else "도달 불가 · 최근접 해"} ({t["pos_err"]*1000:.1f} mm)')); i+=1
while i<len(frames): out.append(label(frames[i],"복귀")); i+=1
iio.mimsave(os.path.join(V1,"img","ik_targets.gif"),[o[::2,::2] if False else o for o in out[::2]],duration=0.08,loop=0)
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
for f in matplotlib.font_manager.fontManager.ttflist:
    if "CJK" in f.name or "Nanum" in f.name or "Malgun" in f.name: plt.rcParams["font.family"]=f.name; break
pick=[k for k in stills if k.split()[0] in ("T1","T3","T7","T10")]
fig,ax=plt.subplots(1,4,figsize=(16,3.9),constrained_layout=True)
for a,k in zip(ax,pick):
    t=next(x for x in T if x["target"]==k); a.imshow(stills[k][20:]); a.axis("off")
    a.set_title(f'{k}\n{"수렴" if t["success"] else "도달 불가 → 최근접 해"} · 오차 {t["pos_err"]*1000:.1f} mm',fontsize=10,loc="left")
fig.savefig(os.path.join(V1,"img","ik_targets.png"),dpi=110); print("frames",len(out)//2,"gif + png saved")

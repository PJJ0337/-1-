# week08 박종진 — 집기 시나리오 GIF (전문가 정책 1회): 단계 이름 · 시간 · 물체 질량 표시, 0.2 s 간격 · 2배속
#   출력: scenarios/img/pick_seed{seed}.gif + 마지막 장면 png     실행: MUJOCO_GL=egl python scenarios/render_pick_gif.py [seed]   (Windows는 그냥 python)
import os,sys
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from expert import run_episode,DT
from pick_env import build,sample_task
seed=int(sys.argv[1]) if len(sys.argv)>1 else 6
m=build(cam=True); W,H=440,330; r=mujoco.Renderer(m,H,W)
cam=mujoco.MjvCamera(); cam.lookat[:]=[0.3,0.0,0.68]; cam.distance=2.2; cam.azimuth=-60; cam.elevation=-28
try: F=ImageFont.truetype(next(x for x in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc","C:/Windows/Fonts/malgun.ttf") if os.path.exists(x)),17)
except StopIteration: F=None
KO=dict(APPROACH="접근(베이스 옆면 붙이기)",ALIGN="정렬",DESCEND="수직 하강",GRASP="파지",LIFT="들기",MOVE="이동(베이스)",PLACE="내려놓기",RELEASE="놓기",RETREAT="후퇴",RETURN="복귀",DONE="완료")
t=sample_task(seed); frames=[]; k=[0]
def cb(d,a,ex):
    k[0]+=1
    if k[0]%10: return       # 0.2 s 간격
    r.update_scene(d,cam); im=Image.fromarray(r.render()); dr=ImageDraw.Draw(im)
    dr.rectangle([0,0,W,46],fill=(255,255,255))
    dr.text((8,3),f"B1 week08 집기 · seed {seed} · {('상자' if t['kind']=='box' else '원기둥')} {t['mass']:.2f} kg",fill=(20,20,20),font=F)
    dr.text((8,24),f"t = {ex.t_total:5.1f} s   단계: {KO.get(ex.stage,ex.stage)}   그리퍼 {2*np.mean(d.qpos[ex.I.fq])*1e3:.0f} mm",fill=(30,60,160),font=F)
    frames.append(im.convert("P",palette=Image.ADAPTIVE,colors=96))
res,d=run_episode(m,seed,on_step=cb)
out=os.path.join(HERE,"img"); os.makedirs(out,exist_ok=True)
frames[0].save(os.path.join(out,f"pick_seed{seed}.gif"),save_all=True,append_images=frames[1:]+[frames[-1]]*8,duration=100,loop=0,optimize=True)
print(res["success"],res["fail"],len(frames),"frames",os.path.getsize(os.path.join(out,f"pick_seed{seed}.gif"))//1024,"KB")

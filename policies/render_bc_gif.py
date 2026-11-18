# week09 박종진 — BC 롤아웃 GIF (보고서 · 저장소 제출용 1개)
#   실행: MUJOCO_GL=egl python policies/render_bc_gif.py joint_dart_prev 2004 [nom|rand]     (Windows는 MUJOCO_GL 없이)
#   출력: policies/img/bc_<tag>_seed<seed>.gif (overview 카메라 320 × 240, 2배속) + _strip.png(장면 6장)
import os,sys,copy
import numpy as np,mujoco,imageio.v2 as imageio
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
import eval_bc as V, pick_env as P

tag=sys.argv[1] if len(sys.argv)>1 else "joint_dart_prev"; seed=int(sys.argv[2]) if len(sys.argv)>2 else 2004; cond=sys.argv[3] if len(sys.argv)>3 else "nom"
m0=P.build(cam=True); mr=copy.deepcopy(m0); mr.vis.global_.offwidth=320; mr.vis.global_.offheight=240
ren=mujoco.Renderer(mr,240,320); frames=[]
if tag=="expert": pol="expert"
else:
    import torch; torch.set_num_threads(1); from train_bc import Policy; pol=Policy(os.path.join(HERE,"ckpt",f"bc_{tag}.pt"))
class R:   # 물리는 m(랜덤화 사본)으로, 렌더는 같은 상태를 mr로 — update_scene는 MjData만 보므로 렌더 모델 = m0 사본이면 충분
    def __init__(s): s.r=ren
    def update_scene(s,d,cam): s.r.update_scene(d,cam)
    def render(s): return s.r.render()
r=V.run(m0,pol,seed,cond=="rand",frames=frames,frame_every=5,renderer=R())
os.makedirs(os.path.join(HERE,"img"),exist_ok=True); out=os.path.join(HERE,"img",f"bc_{tag}_seed{seed}{'_rand' if cond=='rand' else ''}.gif")
imageio.mimsave(out,frames[::2],duration=0.1,loop=0)   # 0.2 s 간격 프레임을 0.1 s로 = 2배속(파일 크기)
# 보고서용 장면 6장 띠(같은 간격)
idx=np.linspace(0,len(frames)-1,6).astype(int); strip=np.concatenate([frames[i] for i in idx],axis=1)
imageio.imwrite(out.replace(".gif","_strip.png"),strip); print(out,len(frames),{k:v for k,v in r.items() if k!="rand"})

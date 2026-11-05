# week07 박종진 — 전신 IK 해 그림: 테이블 안쪽 0.30 m(z 0.75 m) 목표에 대해 ① 팔만(정면, 최근접 해) ② 전신(옆면 붙이기, yaw ≈ −90°)
#   ③ 위에서 아래로 접근(A8) 전신 해 — 테이블(앞 가장자리 x = 0.288, 높이 0.70 m)은 그림용 box(충돌 없음)
#   출력: img/wb_ik_solutions.png     실행: MUJOCO_GL=egl python b1/models/v1/scripts/wb_ik_render.py
import os,sys
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE)
from v1_mjio import _assets
from ik import solve_ik,approach_quat
from wb_ik import solve_wb_ik,apply,qadr
SC=os.path.join(V1,"mjcf","scene.xml"); a=_assets(SC); xml=open(SC,encoding="utf-8").read()
TBL=0.288; TOP=0.70
xml=xml.replace("</worldbody>",f'<geom name="table" type="box" pos="{TBL+0.4} 0.2 {TOP/2}" size="0.4 0.6 {TOP/2}" rgba="0.72 0.55 0.38 0.35" contype="0" conaffinity="0"/>'
                f'<site name="tgt" pos="0 0 0" size="0.03" rgba="1 0.1 0.1 1"/></worldbody>',1)
m=mujoco.MjModel.from_xml_string(xml,assets=a); d=mujoco.MjData(m); S="left"
cases=[("① 팔만 (정면): 테이블 안쪽 0.30 m · z 0.75",[TBL+0.30,0.28,0.75],None,"arm"),
       ("② 전신: 같은 목표",[TBL+0.30,0.28,0.75],None,"wb"),
       ("③ 전신 · 위에서 아래로 접근 (안쪽 0.10 m · z 0.80)",[TBL+0.10,0.28,0.80],[0,0,-1],"wb")]
r=mujoco.Renderer(m,560,620); imgs=[]; AZ=float(os.environ.get('AZ',-60))
for title,p,ax,mode in cases:
    mujoco.mj_resetData(m,d)
    if mode=="arm":
        res=solve_ik(m,d,S,p,None,frame="world"); d.qpos[qadr(m,S,False)]=res.q; mujoco.mj_forward(m,d)
        sub=f"{'수렴' if res.success else '도달 불가'} · 위치 오차 {res.pos_err*1e3:.0f} mm"
    else:
        res=solve_wb_ik(m,d,S,p,ax,table_x=TBL,restarts=8); apply(m,d,S,res)
        sub=f"{'수렴' if res.success else res.status} {res.pos_err*1e3:.1f} mm" + (f" · 축 {np.rad2deg(res.rot_err):.1f}°" if ax else "") + f" · 베이스 yaw {np.rad2deg(res.base[2]):.0f}°, 이동 {res.base_move:.2f} m"
    title=title+"\n"+sub
    m.site_pos[m.site("tgt").id]=p; mujoco.mj_forward(m,d)
    cam=mujoco.MjvCamera(); cam.lookat[:]=[0.25,0.2,0.75]; cam.distance=2.3; cam.azimuth=AZ; cam.elevation=-38
    r.update_scene(d,cam); im=Image.fromarray(r.render()); dr=ImageDraw.Draw(im)
    try: f=ImageFont.truetype(next(x for x in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc","C:/Windows/Fonts/malgun.ttf") if os.path.exists(x)),22)
    except StopIteration: f=None
    dr.rectangle([0,0,560,64],fill=(255,255,255)); dr.text((10,6),title,fill=(20,20,20),font=f); imgs.append(im)
W=Image.new("RGB",(560*3,620),"white")
for i,im in enumerate(imgs): W.paste(im,(560*i,0))
W.save(os.path.join(V1,"img","wb_ik_solutions.png")); print("saved")

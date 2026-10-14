# v0 모델 캡처 (오프스크린). 사용: MUJOCO_GL=osmesa python v0_render.py <scene.xml> <out_dir>
import os,sys,numpy as np,mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if "CJK" in f or "Nanum" in f: plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
scene,OUT=sys.argv[1],sys.argv[2]; os.makedirs(OUT,exist_ok=True)
m=mujoco.MjModel.from_xml_path(scene); d=mujoco.MjData(m)
r=mujoco.Renderer(m,720,960)
def q(n): return m.jnt_qposadr[m.joint(n).id]
poses={
 "a_zero":({}, "영점 (팔 내림 · 팔꿈치 전방 굽힘 방향)"),
 "b_pitch_fwd":({"left_shoulder_pitch_joint":-1.5708,"right_shoulder_pitch_joint":-1.5708},"shoulder_pitch −90° (양팔 앞으로 수평)"),
 "c_roll_abduct":({"left_shoulder_roll_joint":1.2,"right_shoulder_roll_joint":1.2},"shoulder_roll +1.2 rad (외전, 좌우 대칭)"),
 "d_reach":({"left_shoulder_pitch_joint":-0.6,"left_elbow_joint":1.4,"right_shoulder_pitch_joint":-0.3,"right_shoulder_roll_joint":0.5,"right_elbow_joint":0.9},"팔꿈치 전방 굽힘 (elbow +, 영점 재정의 후)"),
}
imgs=[]
for key,(pose,title) in poses.items():
    mujoco.mj_resetData(m,d)
    for j,v in pose.items(): d.qpos[q(j)]=v
    mujoco.mj_forward(m,d)
    cam=mujoco.MjvCamera(); cam.lookat[:]=[0,0,0.95]; cam.distance=2.0; cam.azimuth=145; cam.elevation=-12
    opt=mujoco.MjvOption(); opt.geomgroup[:]=0; opt.geomgroup[1]=1; opt.geomgroup[2]=1
    r.update_scene(d,cam,opt); img=r.render(); imgs.append((img,title,d.ncon))
    plt.imsave(os.path.join(OUT,f"v0_{key}.png"),img)
fig,ax=plt.subplots(1,4,figsize=(19,6.6))
for a,(img,t,nc) in zip(ax,imgs): a.imshow(img); a.set_title(f"{t}\n접촉 {nc}개",fontsize=11); a.axis("off")
fig.suptitle("b1_humanoid_v0 — URDF(함태훈 메시 · inertial + 박종진 골격) → MuJoCo 3.15.0 로드 (robot.xml → robot_actuated.xml → scene.xml)",fontsize=13)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"20261013_v0_mujoco.png"),dpi=110)
# collision(볼록 껍질) 보기
mujoco.mj_resetData(m,d); mujoco.mj_forward(m,d)
cam=mujoco.MjvCamera(); cam.lookat[:]=[0,0,0.95]; cam.distance=2.0; cam.azimuth=145; cam.elevation=-12
opt=mujoco.MjvOption(); opt.geomgroup[:]=0; opt.geomgroup[0]=1; opt.geomgroup[2]=1
r.update_scene(d,cam,opt); plt.imsave(os.path.join(OUT,"v0_collision_hulls.png"),r.render())
print("saved")

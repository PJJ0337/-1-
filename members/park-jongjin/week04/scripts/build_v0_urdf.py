# 골격 URDF + v0 URDF(메시 · CAD 관성) 생성 — 4주차 B1 (박종진)
# 입력: step_to_links.py 결과(meshes/*_visual.stl · *_collision.stl · link_inertials.json, left 팔 기준)
# 출력: <model>/urdf/b1_humanoid_skeleton.urdf, b1_humanoid_v0.urdf, <model>/meshes/*.stl (right = left 거울)
# 사용: python build_v0_urdf.py <step_to_links 출력 폴더> <model 폴더>
import os,sys,json,numpy as np,trimesh
SRC,MODEL=sys.argv[1],sys.argv[2]
os.makedirs(f"{MODEL}/urdf",exist_ok=True); os.makedirs(f"{MODEL}/meshes",exist_ok=True)
INER=json.load(open(f"{SRC}/link_inertials.json",encoding="utf-8"))
A=json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"assumptions_v0.json"),encoding="utf-8"))
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
# ---- 관절 (left, 로봇 좌표 m) — link-split-v0 §3, CAD→robot: x_r=z_c, y_r=x_c+118.15, z_r=y_c
J_XYZ={"shoulder_pitch":(0,0.16975,0),"shoulder_roll":(0,0.1079,-0.0200),"shoulder_yaw":(0,0,-0.1823),"elbow":(0,0,-0.0938)}
J_AX ={"shoulder_pitch":(0,1,0),"shoulder_roll":(1,0,0),"shoulder_yaw":(0,0,1),"elbow":(1,0,0)}
TOOL=(-0.009,0,-0.1989)
def mirror_vec(v): return (v[0],-v[1],v[2])
def mirror_axis(a): return (-a[0],a[1],-a[2])        # 거울면(y=0)에 대한 축벡터(유사벡터) 변환 = ICD 3.4 roll·yaw 부호 반전
def mirror_inertia(I):  # ixx iyy izz ixy ixz iyz
    return [I[0],I[1],I[2],-I[3],I[4],-I[5]]
def f(x): return " ".join(f"{v:.6g}" for v in x)
# ---- 메시: left는 단순화 후 저장, right는 거울
def save_meshes():
    info={}
    for k in ["torso_link"]+[f"left_{a}_link" for a in ARM]:
        v=trimesh.load(f"{SRC}/{k}_visual.stl"); n0=len(v.faces)
        if len(v.faces)>A["mesh"]["visual_max_faces"]:
            v=v.simplify_quadric_decimation(face_count=A["mesh"]["visual_max_faces"])
        c=trimesh.load(f"{SRC}/{k}_collision.stl")
        v.export(f"{MODEL}/meshes/{k}_visual.stl"); c.export(f"{MODEL}/meshes/{k}_collision.stl")
        info[k]=dict(visual_faces_raw=n0,visual_faces=len(v.faces),collision_faces=len(c.faces))
        if k.startswith("left_"):
            r=k.replace("left_","right_")
            for kind,m in (("visual",v),("collision",c)):
                mm=m.copy(); mm.apply_transform(np.diag([1,-1,1,1])); mm.invert() if False else None
                mm.faces=mm.faces[:,::-1]; mm.export(f"{MODEL}/meshes/{r}_{kind}.stl")
            info[r]=dict(info[k],mirrored_from=k)
    return info
def inertial_xml(m,com,I):
    return f'    <inertial>\n      <origin xyz="{f(com)}" rpy="0 0 0"/>\n      <mass value="{m:.5g}"/>\n      <inertia ixx="{I[0]:.6g}" iyy="{I[1]:.6g}" izz="{I[2]:.6g}" ixy="{I[3]:.6g}" ixz="{I[4]:.6g}" iyz="{I[5]:.6g}"/>\n    </inertial>\n'
def box_inertia(m,sx,sy,sz): return [m*(sy*sy+sz*sz)/12,m*(sx*sx+sz*sz)/12,m*(sx*sx+sy*sy)/12,0,0,0]
def combine(parts):  # [(m, com, I6)] → 합성
    M=sum(p[0] for p in parts); C=sum(p[0]*np.array(p[1]) for p in parts)/M; It=np.zeros((3,3))
    for m,c,I in parts:
        Im=np.array([[I[0],I[3],I[4]],[I[3],I[1],I[5]],[I[4],I[5],I[2]]]); r=np.array(c)-C
        It+=Im+m*(r@r*np.eye(3)-np.outer(r,r))
    return M,C,[It[0,0],It[1,1],It[2,2],It[0,1],It[0,2],It[1,2]]
def build(with_geom):
    b=A["base"]; t=A["torso"]; act=A["actuator"]; lim=A["limits"]
    L=[]; w=L.append
    w('<?xml version="1.0"?>\n<!-- 자동 생성: build_v0_urdf.py (week04, 박종진). 값 출처는 assumptions-v0.md / link-split-v0.md -->\n')
    w(f'<robot name="b1_humanoid_{"v0" if with_geom else "skeleton"}">\n')
    w('  <mujoco>\n    <compiler meshdir="../meshes/" fusestatic="false" discardvisual="false" balanceinertia="false"/>\n  </mujoco>\n\n')
    # odom(월드 고정) --planar--> base_link
    w('  <link name="odom"/>\n')
    w('  <joint name="base_planar_joint" type="planar">\n    <parent link="odom"/>\n    <child link="base_link"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n    <axis xyz="0 0 1"/>\n  </joint>\n\n')
    # base_link: 원점 = 바닥면 중심 (ICD 3.1)
    bx,by=b["plate_xy"]; zt=b["plate_top_z"]; zc=b["clearance_z"]
    hb=zt-zc-b["plate_t"]
    w('  <link name="base_link">\n')
    parts=[(b["mass_lower"],(0,0,zc+hb/2),box_inertia(b["mass_lower"],bx,by,hb)),(b["mass_plate"],(0,0,zt-b["plate_t"]/2),box_inertia(b["mass_plate"],bx,by,b["plate_t"]))]
    M,C,I=combine(parts); w(inertial_xml(M,C,I))
    if with_geom:
        for nm,(z,h) in {"lower":(zc+hb/2,hb),"plate":(zt-b["plate_t"]/2,b["plate_t"])}.items():
            g=f'<origin xyz="0 0 {z:.4f}"/>\n      <geometry><box size="{bx} {by} {h:.4f}"/></geometry>'
            w(f'    <visual name="base_{nm}_visual">\n      {g}\n    </visual>\n    <collision name="base_{nm}_collision">\n      {g}\n    </collision>\n')
    w('  </link>\n')
    w(f'  <joint name="torso_fixed_joint" type="fixed">\n    <parent link="base_link"/>\n    <child link="torso_link"/>\n    <origin xyz="0 0 {t["shoulder_z"]}" rpy="0 0 0"/>\n  </joint>\n\n')
    # torso: CAD(J1 RI85 고정측 ×2) + 임시 box
    ti=INER["torso_link"]; tz0=zt-t["shoulder_z"]; tz1=t["box_top_above_shoulder"]; th=tz1-tz0
    parts=[(ti["mass"],ti["com"],ti["inertia"]),(t["box_mass"],(0,0,(tz0+tz1)/2),box_inertia(t["box_mass"],t["box_x"],t["box_y"],th))]
    M,C,I=combine(parts)
    w('  <link name="torso_link">\n'+inertial_xml(M,C,I))
    if with_geom:
        w('    <visual name="torso_actuators_visual">\n      <geometry><mesh filename="torso_link_visual.stl"/></geometry>\n    </visual>\n')
        w('    <collision name="torso_actuators_collision">\n      <geometry><mesh filename="torso_link_collision.stl"/></geometry>\n    </collision>\n')
        g=f'<origin xyz="0 0 {(tz0+tz1)/2:.4f}"/>\n      <geometry><box size="{t["box_x"]} {t["box_y"]} {th:.4f}"/></geometry>'
        w(f'    <visual name="torso_box_visual">\n      {g}\n    </visual>\n    <collision name="torso_box_collision">\n      {g}\n    </collision>\n')
    w('  </link>\n\n')
    # cad_frame: STEP(CAD) 좌표계를 그대로 나타내는 고정 프레임 — CAD → 로봇 변환 하나를 여기 한 곳에 둠 (팀장 10/14)
    import numpy as _np
    from scipy.spatial.transform import Rotation as _Ro
    _R=_np.array(A["cad_to_base"]["R"],float); _c=_np.array(A["cad_to_base"]["c_mm"],float)
    _xyz=_R@(-_c)/1000.0; _rpy=_Ro.from_matrix(_R).as_euler("xyz")
    w(f'  <joint name="cad_frame_joint" type="fixed">\n    <parent link="torso_link"/>\n    <child link="cad_frame"/>\n    <origin xyz="{f(_xyz)}" rpy="{f(_rpy)}"/>\n  </joint>\n')
    w('  <link name="cad_frame">\n    <inertial>\n      <mass value="0.001"/>\n      <inertia ixx="1e-7" iyy="1e-7" izz="1e-7" ixy="0" ixz="0" iyz="0"/>\n    </inertial>\n  </link>\n\n')
    for side in ("left","right"):
        parent="torso_link"
        for a in ARM:
            xyz=J_XYZ[a]; ax=J_AX[a]; li=INER[f"left_{a}_link"]; com=li["com"]; I=li["inertia"]
            if side=="right": xyz=mirror_vec(xyz); ax=mirror_axis(ax); com=mirror_vec(com); I=mirror_inertia(I)
            lo,hi=lim[a]
            if side=="right" and a in ("shoulder_roll","shoulder_yaw","elbow"): pass  # 거울 대칭 정의 → 같은 범위
            w(f'  <joint name="{side}_{a}_joint" type="revolute">\n    <parent link="{parent}"/>\n    <child link="{side}_{a}_link"/>\n')
            rpy=(0,0,0)
            if a=="shoulder_yaw":   # 영점 = 팔 내린 자세 + 팔꿈치 전방 굽힘 → CAD 자세와의 차이(yaw 90°)를 origin rpy로 흡수 (팀장 10/14)
                y0=A["zero_pose"]["left_shoulder_yaw_origin_yaw_rad"]; rpy=(0,0,y0 if side=="left" else -y0)
            w(f'    <origin xyz="{f(xyz)}" rpy="{f(rpy)}"/>\n    <axis xyz="{f(ax)}"/>\n')
            w(f'    <limit lower="{lo}" upper="{hi}" effort="{act["effort_peak"]}" velocity="{act["velocity"]}"/>\n')
            w(f'    <dynamics damping="{act["damping"]}" friction="{act["friction"]}"/>\n  </joint>\n')
            w(f'  <link name="{side}_{a}_link">\n'+inertial_xml(li["mass"],com,I))
            if with_geom:
                for kind in ("visual","collision"):
                    w(f'    <{kind} name="{side}_{a}_{kind}">\n      <geometry><mesh filename="{side}_{a}_link_{kind}.stl"/></geometry>\n    </{kind}>\n')
            w('  </link>\n')
            parent=f"{side}_{a}_link"
        tx=TOOL if side=="left" else mirror_vec(TOOL)
        w(f'  <joint name="{side}_tool0_joint" type="fixed">\n    <parent link="{parent}"/>\n    <child link="{side}_tool0"/>\n    <origin xyz="{f(tx)}" rpy="0 0 0"/>\n  </joint>\n')
        w(f'  <link name="{side}_tool0">\n    <inertial>\n      <mass value="0.001"/>\n      <inertia ixx="1e-7" iyy="1e-7" izz="1e-7" ixy="0" ixz="0" iyz="0"/>\n    </inertial>\n  </link>\n\n')
    w('</robot>\n')
    return "".join(L)
info=save_meshes()
open(f"{MODEL}/urdf/b1_humanoid_skeleton.urdf","w",encoding="utf-8").write(build(False))
open(f"{MODEL}/urdf/b1_humanoid_v0.urdf","w",encoding="utf-8").write(build(True))
json.dump(info,open(f"{MODEL}/meshes/mesh_info.json","w"),indent=1)
print(json.dumps(info,indent=0)[:600])

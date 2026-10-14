# 함태훈 메시 · inertial(함태훈 저장소 f9c9eff sim/robot/) → 박종진 골격 링크 프레임으로 옮겨 build_v0_urdf.py 입력 형식으로 저장
# 두 URDF 모두 cad_frame을 갖고 링크 축 방향이 같으므로(회전 차 0) 링크 프레임 차이는 평행이동 dp뿐:
#   COM_B1 = COM_TH + dp,  I(COM 기준) 동일,  메시 정점 v_B1 = v_TH + dp     (dp = 함태훈 링크 원점을 B1 링크 프레임으로 표현)
# 사용: python merge_taehoon.py <함태훈 our_robot_v0.urdf> <함태훈 link_mass_props.csv> <B1 b1_humanoid_v0.urdf> <out_dir>
import os,sys,csv,json,ast,numpy as np,mujoco,trimesh,xml.etree.ElementTree as ET
TH,THCSV,B1,OUT=sys.argv[1:5]; os.makedirs(OUT,exist_ok=True)
mh=mujoco.MjModel.from_xml_path(TH); dh=mujoco.MjData(mh); mujoco.mj_forward(mh,dh)
mb=mujoco.MjModel.from_xml_path(B1); db=mujoco.MjData(mb); mujoco.mj_forward(mb,db)
def rel(m,d,b):
    c=m.body("cad_frame").id; i=m.body(b).id; Rc=d.xmat[c].reshape(3,3)
    return Rc.T@(d.xpos[i]-d.xpos[c]), Rc.T@d.xmat[i].reshape(3,3)
root=ET.parse(TH).getroot(); meshdir=os.path.join(os.path.dirname(os.path.abspath(TH)),"meshes")
out={}; rep=[]
for s in ("left","right"):
    for a in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"):
        k=f"{s}_{a}_link"; ph,Rh=rel(mh,dh,k); pb,Rb=rel(mb,db,k)
        dR=Rb.T@Rh; assert np.abs(dR-np.eye(3)).max()<1e-6, k
        dp=Rb.T@(ph-pb)
        L=[l for l in root.findall("link") if l.get("name")==k][0]; ine=L.find("inertial")
        com=np.array([float(x) for x in ine.find("origin").get("xyz").split()]); m=float(ine.find("mass").get("value"))
        I=ine.find("inertia"); I6=[float(I.get(n)) for n in ("ixx","iyy","izz","ixy","ixz","iyz")]
        out[k]=dict(mass=m,com=list(com+dp),inertia=I6,source="함태훈 f9c9eff our_robot_v0.urdf",dp_mm=list(np.round(dp*1000,2)))
        for kind in ("visual","collision"):
            t=trimesh.load(os.path.join(meshdir,f"{k}_{kind}.stl")); t.apply_translation(dp); t.export(os.path.join(OUT,f"{k}_{kind}.stl"))
        rep.append((k,m,np.round(dp*1000,2)))
# torso: 함태훈 CAD 부품(J1 RI85 고정측 등)만 = csv mass_occ/com_occ/I_occ (B1 torso 프레임과 원점 차 0.01 mm)
row=[r for r in csv.DictReader(open(THCSV,encoding="utf-8-sig")) if r["link"]=="torso_link"][0]
Iq=np.array(ast.literal_eval(row["I_occ"])); pt,_=rel(mh,dh,"torso_link"); pbt,_=rel(mb,db,"torso_link")
out["torso_link"]=dict(mass=float(row["mass_occ"]),com=list(np.array(ast.literal_eval(row["com_occ"]))+(pt-pbt)),
    inertia=[Iq[0,0],Iq[1,1],Iq[2,2],Iq[0,1],Iq[0,2],Iq[1,2]],source="함태훈 link_mass_props.csv (CAD 부품만, 몸통 box는 B1 가정 S11)")
for kind in ("visual","collision"):
    t=trimesh.load(os.path.join(meshdir,f"torso_link_{kind}.stl")); t.apply_translation(pt-pbt); t.export(os.path.join(OUT,f"torso_link_{kind}.stl"))
json.dump(out,open(os.path.join(OUT,"link_inertials.json"),"w"),indent=1,ensure_ascii=False)
for r in rep: print(r)
print("torso",out["torso_link"]["mass"])

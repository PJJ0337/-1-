# STEP → 링크별 (visual STL, collision convex hull STL, 질량·COM·관성 [재질 가정]) — 링크 프레임(로봇 좌표, m)
# 사용: python step_to_links.py <arm.stp> <out_dir>
# 링크 배정은 link-split-v0.md(step_link_split.py)와 같은 규칙, 재질은 부품명으로 가정(아래 MAT)
import os,sys,json,numpy as np,trimesh
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_tree.py"),encoding="utf-8").read().split("rows=[]")[0])
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
OUT=sys.argv[2]; os.makedirs(OUT,exist_ok=True)
# ---- CAD(mm) → 로봇(REP 103) 고정 변환 하나: assumptions_v0.json "cad_to_base" (CAD 축을 직접 쓰지 않음, 팀장 10/14)
_T=json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"assumptions_v0.json"),encoding="utf-8"))["cad_to_base"]
R=np.array(_T["R"],float); C=np.array(_T["c_mm"],float); XC=C[0]
def side_of(p_cad):  # 좌/우 = 변환 후 +y 부호
    return "L" if (R@(np.asarray(p_cad,float)-C))[1]>0 else "R"
def mirror_cad(p):   # 로봇 y=0 면(좌우 대칭면)에 대한 거울을 CAD 좌표로
    q=R@(np.asarray(p,float)-C); q[1]=-q[1]; return R.T@q+C
# ---- 관절 점 (CAD mm, left = 변환 후 +y 쪽 팔, 이상화: J3·J4 z=0, 기울기 무시 — link-split-v0 §3·§4)
JP={"shoulder_pitch":(51.6,0,0),"shoulder_roll":(159.5,-20.0,0),"shoulder_yaw":(159.5,-202.3,0),"elbow":(159.5,-296.1,0)}
LINK_ORIGIN={"shoulder_pitch_link":JP["shoulder_pitch"],"shoulder_roll_link":JP["shoulder_roll"],"shoulder_yaw_link":JP["shoulder_yaw"],"elbow_link":JP["elbow"]}
# ---- 재질 가정 (g/cm3): 강재 = 기어·축·베어링·키, 나머지 Al. PCB·자석 등 세부 무시
STEEL=("Gear","InvoluteGear","shaft","JIS","ISO","GSRAU","key","MRA","Elastoferrite","Prirobnica","sun adapter","planet carrier")
def density(part):
    return 7.85 if any(k.lower() in part.lower() for k in STEEL) else 2.70
# visual에서 뺄 액추에이터 내부 부품
INTERNAL=("Gear","shaft","JIS","ISO","GSRAU","key","MRA","Elastoferrite","Prirobnica","MB039","sun adapter","encoder plate","planet carrier","planet")
M={"어깨 브래킷1":"shoulder_pitch_link","어깨 브래킷2":"shoulder_pitch_link","어깨 브래킷1(반대)":"shoulder_pitch_link","어깨 브래킷2(반대)":"shoulder_pitch_link",
 "C브래킷1(수정본)":"shoulder_roll_link","C브래킷2(수정본)":"shoulder_roll_link","상완원통 일체형":"shoulder_roll_link",
 "상완 칼라1":"shoulder_yaw_link","상완 칼라2":"shoulder_yaw_link","전완요크1":"elbow_link","전완요크2":"elbow_link","그리퍼 플랜지":"elbow_link"}
RI={1:("L",1),2:("L",2),3:("L",3),4:("L",4),6:("R",1),5:("R",2),7:("R",3),8:("R",4)}
chain=["torso_link","shoulder_pitch_link","shoulder_roll_link","shoulder_yaw_link","elbow_link"]
drv={1:"shoulder_pitch_link",2:"shoulder_pitch_link",3:"shoulder_roll_link",4:"shoulder_roll_link",5:"elbow_link",6:"elbow_link"}
brg={1:"shoulder_pitch_link",3:"shoulder_pitch_link",2:"shoulder_yaw_link",4:"shoulder_yaw_link"}
leaves=[]  # (side, link, partname, shape)
def collect(lab,loc,side,link,ri_out=None,top=None):
    ref=TDF_Label()
    if st.IsReference_s(lab): st.GetReferredShape_s(lab,ref); l2=loc.Multiplied(st.GetLocation_s(lab))
    else: ref=lab; l2=loc
    pn=name(ref)
    if st.IsAssembly_s(ref):
        sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub)
        for k in range(1,sub.Length()+1):
            s=sub.Value(k)
            r2=TDF_Label(); st.GetReferredShape_s(s,r2)
            lk=link
            if ri_out is not None and name(r2).startswith(("output","ring2","cover plate1")): lk=ri_out
            collect(s,l2,side,lk,None if ri_out is None or lk==ri_out else ri_out,top)
    else:
        leaves.append((side,link,pn,st.GetShape_s(ref).Moved(l2),top))
tops=TDF_LabelSequence(); st.GetFreeShapes(tops); root=tops.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref)
    inst=name(c); base,num=inst.rsplit(":",1); num=int(num); part=name(ref)
    b=Bnd_Box(); BRepBndLib.Add_s(st.GetShape_s(ref).Moved(st.GetLocation_s(c)),b); cx=(b.CornerMin().X()+b.CornerMax().X())/2
    side=side_of(((b.CornerMin().X()+b.CornerMax().X())/2,(b.CornerMin().Y()+b.CornerMax().Y())/2,(b.CornerMin().Z()+b.CornerMax().Z())/2))
    if "RI85" in part:
        s,j=RI[num]; collect(c,TopLoc_Location(),s,chain[j-1],ri_out=chain[j],top=inst); continue
    if base in M: lk=M[base]
    elif base.startswith("모터드라이버"): lk=drv[num]
    elif base.startswith("JIS B 1521 6810"): lk=brg[num]
    else: raise SystemExit("미배정 "+inst)
    collect(c,TopLoc_Location(),side,lk,top=inst)
print("leaf solids",len(leaves))
def tris_of(shp):
    BRepMesh_IncrementalMesh(shp,0.3,False,0.35,True)
    V=[];F=[];n=0
    ex=TopExp_Explorer(shp,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face(ex.Current()); L=TopLoc_Location(); t=BRep_Tool.Triangulation_s(f,L)
        if t is not None:
            tr=L.Transformation(); rev=f.Orientation()==1
            for k in range(1,t.NbNodes()+1):
                p=t.Node(k).Transformed(tr); V.append((p.X(),p.Y(),p.Z()))
            for k in range(1,t.NbTriangles()+1):
                a,b_,c_=t.Triangle(k).Get(); F.append((n+a-1,n+c_-1,n+b_-1) if rev else (n+a-1,n+b_-1,n+c_-1))
            n=len(V)
        ex.Next()
    return np.array(V),np.array(F,int)
links={}
for side,link,pn,shp,top in leaves:
    key=("torso_link" if link=="torso_link" else ("left_" if side=="L" else "right_")+link)
    d=links.setdefault(key,dict(side=side,base=link,m=0,mc=np.zeros(3),parts=[],V=[],F=[],nv=0,Ilist=[]))
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(shp,g)
    vol=g.Mass()
    if vol<=1e-6: continue
    rho=density(pn); m=rho*vol/1e6  # mm3*g/cm3 → kg : vol[mm3]/1000=cm3, *rho g, /1000 kg
    c=g.CentreOfMass(); c=np.array([c.X(),c.Y(),c.Z()])
    mi=g.MatrixOfInertia(); I=np.array([[mi.Value(i,j) for j in (1,2,3)] for i in (1,2,3)])*rho/1e6*1e-6  # kg·m2 (mm5*g/cm3)
    d["m"]+=m; d["mc"]+=m*c; d["Ilist"].append((m,c,I)); d["parts"].append(pn)
    if not any(k.lower() in pn.lower() for k in INTERNAL):
        V,F=tris_of(shp)
        if len(F): d["V"].append(V); d["F"].append(F+d["nv"]); d["nv"]+=len(V)
summary={}
for key,d in sorted(links.items()):
    m=d["m"]; com=d["mc"]/m
    Ic=np.zeros((3,3))
    for mi_,c,I in d["Ilist"]:
        r=(c-com)/1000.0
        Ic+=I+mi_*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
    # 링크 원점 (CAD mm)
    if d["base"]=="torso_link": O=C.copy()
    else:
        O=np.array(LINK_ORIGIN[d["base"]],float)
        if d["side"]=="R": O=mirror_cad(O)   # 거울 위치 (로봇 y 부호 반전)
    com_r=R@(com-O)/1000.0
    I_r=R@Ic@R.T
    V=np.vstack(d["V"]); F=np.vstack(d["F"])
    Vr=(R@(V-O).T).T/1000.0
    vm=trimesh.Trimesh(Vr,F,process=True)
    vm.export(os.path.join(OUT,f"{key}_visual.stl"))
    hull=vm.convex_hull; hull.export(os.path.join(OUT,f"{key}_collision.stl"))
    w=np.linalg.eigvalsh(I_r)
    summary[key]=dict(mass=round(m,4),com=[round(x,5) for x in com_r],inertia=[float(I_r[0,0]),float(I_r[1,1]),float(I_r[2,2]),float(I_r[0,1]),float(I_r[0,2]),float(I_r[1,2])],
        principal=[float(x) for x in w],triangle_ok=bool(w[0]+w[1]>=w[2]),visual_faces=len(vm.faces),hull_faces=len(hull.faces),watertight=bool(vm.is_watertight),
        hull_volume_cm3=round(hull.volume*1e6,1),n_parts=len(d["parts"]),origin_cad_mm=[round(x,2) for x in O])
    print(key,summary[key]["mass"],summary[key]["com"],summary[key]["visual_faces"],summary[key]["hull_faces"])
json.dump(summary,open(os.path.join(OUT,"link_inertials.json"),"w"),indent=1,ensure_ascii=False)

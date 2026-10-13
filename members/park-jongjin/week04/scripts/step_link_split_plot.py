# 링크 분할 결과를 색으로 구분한 그림 (CAD 좌표, mm). 사용: python step_link_split_plot.py <arm.stp> <out.png>
import os,sys,numpy as np
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_tree.py"),encoding="utf-8").read().split("rows=[]")[0])
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRep import BRep_Tool
from OCP.TopoDS import TopoDS
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if "Nanum" in f or "CJK" in f: plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
top=TDF_LabelSequence(); st.GetFreeShapes(top); root=top.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
import csv
link_of={}
for r in csv.DictReader(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","results","link_split_parts.csv"),encoding="utf-8")):
    link_of[(r["step_instance"],r["step_part"]=="output-v1")]=r["link"]
COL={"torso_link":"#888888","shoulder_pitch_link":"#2a7ab0","shoulder_roll_link":"#e08a1e","shoulder_yaw_link":"#3a9a4a","elbow_link":"#c0392b"}
tris={k:[] for k in COL}
def mesh(shp,link):
    BRepMesh_IncrementalMesh(shp,1.5,False,0.5,True)
    ex=TopExp_Explorer(shp,TopAbs_FACE)
    while ex.More():
        f=TopoDS.Face(ex.Current()); L=TopLoc_Location(); t=BRep_Tool.Triangulation_s(f,L)
        if t is not None:
            tr=L.Transformation()
            P=[t.Node(i).Transformed(tr) for i in range(1,t.NbNodes()+1)]
            P=np.array([[p.X(),p.Y(),p.Z()] for p in P])
            for i in range(1,t.NbTriangles()+1):
                a,b,c=t.Triangle(i).Get(); tris[link].append(P[[a-1,b-1,c-1]])
        ex.Next()
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref); loc=st.GetLocation_s(c); inst=name(c)
    if "RI85" in name(ref):
        sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub)
        for k in range(1,sub.Length()+1):
            sl=sub.Value(k); r2=TDF_Label(); st.GetReferredShape_s(sl,r2)
            out=name(r2).startswith(("output","ring2","cover plate1"))
            if not (out or name(r2).startswith(("housing","RI85-PH","encoder cover"))): continue
            mesh(st.GetShape_s(r2).Moved(loc.Multiplied(st.GetLocation_s(sl))),link_of[(inst,out)])
    else:
        mesh(st.GetShape_s(ref).Moved(loc),link_of[(inst,False)])
fig=plt.figure(figsize=(12,7.5)); ax=fig.add_subplot(111,projection="3d")
for k,v in tris.items():
    if v: ax.add_collection3d(Poly3DCollection(v,facecolor=COL[k],edgecolor="none",alpha=0.9,label=k))
ax.set_xlim(-480,240); ax.set_ylim(-520,80); ax.set_zlim(-300,300)
ax.set_box_aspect((720,600,600)); ax.view_init(elev=20,azim=-60,vertical_axis="y")
ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm]"); ax.set_zlabel("z [mm]")
from matplotlib.patches import Patch
ax.legend(handles=[Patch(color=c,label=k) for k,c in COL.items()],loc="upper left")
ax.set_title("양팔로봇 최종버전.stp — 링크 분할 v0 (RI85: 고정측=부모, output-v1=자식)\n회색 torso_link = J1 RI85 고정측만 (몸통 CAD 없음)")
plt.tight_layout(); plt.savefig(sys.argv[2],dpi=130)
print({k:len(v) for k,v in tris.items()})

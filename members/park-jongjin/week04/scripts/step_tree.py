# STEP 어셈블리 트리 + 부품별 체적·COM·바운딩박스 (mm). 사용: python step_tree.py <in.stp> <out.pkl>  (pip install cadquery-ocp)
import sys
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location
import pickle
doc=TDocStd_Document(TCollection_ExtendedString("doc"))
r=STEPCAFControl_Reader(); r.SetNameMode(True)
assert r.ReadFile(sys.argv[1])==IFSelect_RetDone
r.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
def name(l):
    a=TDataStd_Name()
    if l.FindAttribute(TDataStd_Name.GetID_s(),a): return a.Get().ToExtString()
    return "?"
rows=[]
def walk(lab,loc,depth,path):
    ref=TDF_Label()
    nm=name(lab)
    if st.IsReference_s(lab):
        st.GetReferredShape_s(lab,ref); l2=loc.Multiplied(st.GetLocation_s(lab)); proto=ref
        pnm=name(ref)
    else:
        proto=lab; l2=loc; pnm=nm
    if st.IsAssembly_s(proto):
        comps=TDF_LabelSequence(); st.GetComponents_s(proto,comps)
        rows.append((depth,nm,pnm,"ASM",None,None,None))
        for i in range(1,comps.Length()+1): walk(comps.Value(i),l2,depth+1,path+[nm])
    else:
        shp=st.GetShape_s(proto).Moved(l2)
        g=GProp_GProps(); BRepGProp.VolumeProperties_s(shp,g)
        b=Bnd_Box(); BRepBndLib.Add_s(shp,b); mn=b.CornerMin(); mx=b.CornerMax(); bb=(mn.X(),mn.Y(),mn.Z(),mx.X(),mx.Y(),mx.Z())
        c=g.CentreOfMass()
        rows.append((depth,nm,pnm,"PART",g.Mass(),(c.X(),c.Y(),c.Z()),bb))
top=TDF_LabelSequence(); st.GetFreeShapes(top)
for i in range(1,top.Length()+1): walk(top.Value(i),TopLoc_Location(),0,[])
pickle.dump(rows,open(sys.argv[2],"wb"))
for d,n,p,t,v,c,bb in rows:
    if t=="ASM": print("  "*d+f"[{p}] ({n})")
    else: print("  "*d+f"{p} | {n} | V={v/1e3:.1f}cm3 c=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) bb=({bb[0]:.0f},{bb[1]:.0f},{bb[2]:.0f})-({bb[3]:.0f},{bb[4]:.0f},{bb[5]:.0f})")

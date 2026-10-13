# 부품 간 접촉(거리<0.5 mm) → 링크 연결 판정. RI85는 고정측(HOUS)/출력측(OUT) 분리. 사용: python step_contacts.py <arm.stp> R|L
import sys,numpy as np
import os; exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_tree.py"),encoding="utf-8").read().split("rows=[]")[0])
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
top=TDF_LabelSequence(); st.GetFreeShapes(top); root=top.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
items={}
side=sys.argv[2]  # 'R' x>-100 or 'L'
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref); loc=st.GetLocation_s(c)
    shp=st.GetShape_s(ref).Moved(loc)
    b=Bnd_Box(); BRepBndLib.Add_s(shp,b); cx=(b.CornerMin().X()+b.CornerMax().X())/2
    if (cx>-115)!=(side=='R'): continue
    nm=name(c)
    if "RI85" in name(ref):
        sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub)
        bb=BRep_Builder(); H=TopoDS_Compound(); bb.MakeCompound(H)
        for j in range(1,sub.Length()+1):
            s=sub.Value(j); r2=TDF_Label(); st.GetReferredShape_s(s,r2)
            sh=st.GetShape_s(r2).Moved(loc.Multiplied(st.GetLocation_s(s)))
            if name(r2).startswith(("output","ring2","cover plate1")): items[nm+"/OUT"]=sh
            elif name(r2).startswith(("housing","RI85-PH","encoder cover")): bb.Add(H,sh)
        items[nm+"/HOUS"]=H
    else: items[nm]=shp
names=list(items)
BB={}
for k,v in items.items():
    b=Bnd_Box(); BRepBndLib.Add_s(v,b); BB[k]=(b.CornerMin().X(),b.CornerMin().Y(),b.CornerMin().Z(),b.CornerMax().X(),b.CornerMax().Y(),b.CornerMax().Z())
def ov(p,q,t=1.0): return all(p[i]-t<=q[i+3] and q[i]-t<=p[i+3] for i in range(3))
for a in range(len(names)):
    for b in range(a+1,len(names)):
        if names[a].split("/")[0]==names[b].split("/")[0]: continue
        if not ov(BB[names[a]],BB[names[b]]): continue
        d=BRepExtrema_DistShapeShape(items[names[a]],items[names[b]]); 
        if d.IsDone() and d.Value()<0.5: print(f"{names[a]:<28s} -- {names[b]:<28s} {d.Value():.2f}",flush=True)

# RI85 구동기별 회전축(출력 디스크 대칭축)·출력면 위치. 사용: python step_joint_axes.py <arm.stp>
import sys,numpy as np
import os; exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_tree.py"),encoding="utf-8").read().split("rows=[]")[0])
top=TDF_LabelSequence(); st.GetFreeShapes(top); root=top.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
def props(shp):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(shp,g); c=g.CentreOfMass(); m=g.MatrixOfInertia()
    I=np.array([[m.Value(i,j) for j in (1,2,3)] for i in (1,2,3)])
    return g.Mass(),np.array([c.X(),c.Y(),c.Z()]),I
res=[]
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref)
    if "RI85" not in name(ref): continue
    loc=st.GetLocation_s(c)
    shp=st.GetShape_s(ref).Moved(loc)
    V,com,I=props(shp)
    w,v=np.linalg.eigh(I)
    # symmetric axis: eigenvalue most different from other two
    d=[abs(w[k]-np.mean(np.delete(w,k))) for k in range(3)]
    k=int(np.argmax(d)); ax=v[:,k]
    # output part com
    sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub)
    oc=None
    for j in range(1,sub.Length()+1):
        s=sub.Value(j); r2=TDF_Label(); st.GetReferredShape_s(s,r2)
        if name(r2).startswith("output"):
            sh=st.GetShape_s(r2).Moved(loc.Multiplied(st.GetLocation_s(s)))
            _,oc,Io=props(sh); w2,v2=np.linalg.eigh(Io); d2=[abs(w2[q]-np.mean(np.delete(w2,q))) for q in range(3)]; oax=v2[:,int(np.argmax(d2))]
    # orient axis toward output
    ax=oax
    if np.dot(oc-com,ax)<0: ax=-ax
    # axis point: project com onto axis line => use com; output face center
    print(f"{name(c):<16s} com={np.round(com,1)} axis(out)={np.round(ax,3)} outcom={np.round(oc,1)} out_offset={np.dot(oc-com,ax):.1f}mm")

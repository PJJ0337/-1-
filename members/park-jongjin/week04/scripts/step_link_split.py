# 링크 분할 → link_split_parts.csv + 링크별 체적·COM 합계. 사용: python step_link_split.py <arm.stp>
import sys,numpy as np,csv,json
import os; exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_tree.py"),encoding="utf-8").read().split("rows=[]")[0])
top=TDF_LabelSequence(); st.GetFreeShapes(top); root=top.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
def props(shp):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(shp,g); c=g.CentreOfMass()
    return g.Mass(),np.array([c.X(),c.Y(),c.Z()])
# instance -> (side, link)
A,B="A(x+)","B(x-)"
M={ # instance name : link suffix
 "어깨 브래킷1":"shoulder_pitch_link","어깨 브래킷2":"shoulder_pitch_link","어깨 브래킷1(반대)":"shoulder_pitch_link","어깨 브래킷2(반대)":"shoulder_pitch_link",
 "C브래킷1(수정본)":"shoulder_roll_link","C브래킷2(수정본)":"shoulder_roll_link","상완원통 일체형":"shoulder_roll_link",
 "상완 칼라1":"shoulder_yaw_link","상완 칼라2":"shoulder_yaw_link",
 "전완요크1":"elbow_link","전완요크2":"elbow_link","그리퍼 플랜지":"elbow_link",
}
# RI85 instance -> (joint idx) ; housing goes to parent link, output to child
RI={1:("A",1),2:("A",2),3:("A",3),4:("A",4),6:("B",1),5:("B",2),7:("B",3),8:("B",4)}
chain=["torso_link","shoulder_pitch_link","shoulder_roll_link","shoulder_yaw_link","elbow_link"]
drv={1:"shoulder_pitch_link",2:"shoulder_pitch_link",3:"shoulder_roll_link",4:"shoulder_roll_link",5:"elbow_link",6:"elbow_link"}
brg={1:"shoulder_pitch_link",3:"shoulder_pitch_link",2:"shoulder_yaw_link",4:"shoulder_yaw_link"}
rows=[]
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref); loc=st.GetLocation_s(c)
    inst=name(c); base,num=inst.rsplit(":",1); num=int(num); part=name(ref)
    shp=st.GetShape_s(ref).Moved(loc); V,com=props(shp)
    side="A" if com[0]>-115 else "B"
    if "RI85" in part:
        s,j=RI[num]
        sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub)
        Vo=0;co=np.zeros(3)
        for k in range(1,sub.Length()+1):
            sl=sub.Value(k); r2=TDF_Label(); st.GetReferredShape_s(sl,r2)
            if name(r2).startswith("output"):
                Vo,co=props(st.GetShape_s(r2).Moved(loc.Multiplied(st.GetLocation_s(sl))))
        Vh=V-Vo; ch=(com*V-co*Vo)/Vh
        rows.append([s,chain[j-1],inst,part+" (출력부 제외: housing·기어·베어링·엔코더)","구동기 J%d 고정측"%j,Vh,*ch])
        rows.append([s,chain[j],inst,"output-v1","구동기 J%d 출력측"%j,Vo,*co])
        continue
    if base in M: link=M[base]; role="구조"
    elif base.startswith("모터드라이버"): link=drv[num]; role="전장(예상)"
    elif base.startswith("JIS B 1521 6810"): link=brg[num]; role="보조 베어링(반대편 지지)"
    else: link="?"; role="?"
    rows.append([side,link,inst,part,role,V,*com])
with open("link_split_parts.csv","w",newline="",encoding="utf-8") as f:
    w=csv.writer(f); w.writerow(["side","link","step_instance","step_part","role","volume_cm3","com_x_mm","com_y_mm","com_z_mm"])
    for r in rows: w.writerow(r[:5]+[round(r[5]/1e3,2)]+[round(x,1) for x in r[6:]])
# summary
S={}
for r in rows:
    k=(r[0],r[1]); S.setdefault(k,[0,np.zeros(3),[]]); S[k][0]+=r[5]; S[k][1]+=r[5]*np.array(r[6:9]); S[k][2].append(r[2])
for k in sorted(S):
    v,m,l=S[k]; print(k, f"V={v/1e3:.1f}cm3 Al={v/1e3*2.70/1e3:.3f}kg com={np.round(m/v,1)} n={len(l)}")
print(sum(r[5] for r in rows)/1e3)

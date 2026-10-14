# joint 원점 검증표: CAD(STEP) 실측 관절 축 vs URDF 공칭(축 직교 · 좌우 대칭) — 팀장 10/14 지시
# 실측: 각 RI85 출력부(output-v1) 디스크 대칭축 · 중심 (STEP)  /  공칭: URDF를 MuJoCo로 읽고 CAD 자세로 둔 joint 축 · 앵커
# 사용: python joint_origin_check.py <arm.stp> <b1_humanoid_v0.urdf> <out.csv> <out.md>
import os,sys,csv,numpy as np,mujoco
STP,URDF,OCSV,OMD=sys.argv[1:5]
sys.argv=[sys.argv[0],STP,"/tmp/_jx"]
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_to_links.py"),encoding="utf-8").read().split("links={}")[0])
A=json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"assumptions_v0.json"),encoding="utf-8"))
JN={1:"shoulder_pitch",2:"shoulder_roll",3:"shoulder_yaw",4:"elbow"}
# 실측 (CAD mm)
meas={}
tops=TDF_LabelSequence(); st.GetFreeShapes(tops); root=tops.Value(1)
comps=TDF_LabelSequence(); st.GetComponents_s(root,comps)
for i in range(1,comps.Length()+1):
    c=comps.Value(i); ref=TDF_Label(); st.GetReferredShape_s(c,ref)
    if "RI85" not in name(ref): continue
    inst=name(c); num=int(inst.rsplit(":",1)[1]); s_,j=RI[num]; loc=st.GetLocation_s(c)
    sub=TDF_LabelSequence(); st.GetComponents_s(ref,sub); allg=GProp_GProps(); BRepGProp.VolumeProperties_s(st.GetShape_s(ref).Moved(loc),allg)
    ca=allg.CentreOfMass(); ca=np.array([ca.X(),ca.Y(),ca.Z()])
    for k in range(1,sub.Length()+1):
        sl=sub.Value(k); r2=TDF_Label(); st.GetReferredShape_s(sl,r2)
        if name(r2).startswith("output"):
            g=GProp_GProps(); BRepGProp.VolumeProperties_s(st.GetShape_s(r2).Moved(loc.Multiplied(st.GetLocation_s(sl))),g)
            cc=g.CentreOfMass(); cc=np.array([cc.X(),cc.Y(),cc.Z()]); mi=g.MatrixOfInertia()
            I=np.array([[mi.Value(a,b) for b in (1,2,3)] for a in (1,2,3)]); w,v=np.linalg.eigh(I)
            ax=v[:,int(np.argmax([abs(w[q]-np.mean(np.delete(w,q))) for q in range(3)]))]
            if np.dot(cc-ca,ax)<0: ax=-ax
            meas[("left" if s_=="L" else "right",JN[j])]=(cc,ax,inst)
# 공칭: URDF, CAD 자세 = shoulder_yaw q 를 영점 rpy 만큼 되돌림
m=mujoco.MjModel.from_xml_path(URDF); d=mujoco.MjData(m)
y0=A["zero_pose"]["left_shoulder_yaw_origin_yaw_rad"]
for s in ("left","right"): d.qpos[m.jnt_qposadr[m.joint(f"{s}_shoulder_yaw_joint").id]]=-y0
mujoco.mj_forward(m,d)
cf=m.body("cad_frame").id; Rw=d.xmat[cf].reshape(3,3); pw=d.xpos[cf]
rows=[]
for (s,jn),(cc,ax,inst) in sorted(meas.items(),key=lambda t:(t[0][0],list(JN.values()).index(t[0][1]))):
    P=pw+Rw@(cc/1000); a=Rw@ax                     # 실측 → 로봇(월드)
    jid=m.joint(f"{s}_{jn}_joint").id; Pn=d.xanchor[jid]; an=d.xaxis[jid]
    ang=np.degrees(np.arccos(min(1,abs(np.dot(a,an)))))
    r=Pn-P; off=np.linalg.norm(r-np.dot(r,a)*a)*1000   # 공칭 앵커 ~ 실측 축선 거리
    sign="같음" if np.dot(a,an)>0 else "반대(부호 규약)"
    rows.append([f"{s}_{jn}_joint",inst,*np.round(P*1000,1),*np.round(a,4),*np.round(Pn*1000,1),*np.round(an,4),round(ang,3),round(off,2),sign])
hdr=["joint","STEP 구동기","실측 x","y","z (mm, base)","실측 축 x","y","z","공칭 x","y","z","공칭 축 x","y","z","축 각도차 °","축선 거리 mm","+방향"]
with open(OCSV,"w",newline="",encoding="utf-8") as fh: w_=csv.writer(fh); w_.writerow(hdr); w_.writerows(rows)
with open(OMD,"w",encoding="utf-8") as fh:
    fh.write("| joint | STEP 구동기 | 축 각도차 [°] | 축선 거리 [mm] | 실측 +방향 vs URDF |\n|---|---|---|---|---|\n")
    for r in rows: fh.write(f"| `{r[0]}` | {r[1]} | {r[14]:.2f} | {r[15]:.1f} | {r[16]} |\n")
for r in rows: print(r[0],r[1],"ang",r[14],"off",r[15],r[16])

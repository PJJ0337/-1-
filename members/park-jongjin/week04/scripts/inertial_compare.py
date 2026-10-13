# 링크 1개 inertial 3자 비교 (커리큘럼 수요일): OCCT 정밀 B-rep(=FreeCAD 커널) vs trimesh(메시) vs BOM
# 사용: python inertial_compare.py <arm.stp> <link: left_elbow_link> <out.md>
import os,sys,numpy as np,trimesh
STP,LINK,OUTMD=sys.argv[1],sys.argv[2],sys.argv[3]
sys.argv=[sys.argv[0],STP,"/tmp/_x"]
src=open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"step_to_links.py"),encoding="utf-8").read().split("links={}")[0]
exec(src)
side="L" if LINK.startswith("left_") else "R"; base=LINK.split("_",1)[1]
sel=[(pn,shp) for s,lk,pn,shp,top in leaves if s==side and lk==base]
O=np.array(LINK_ORIGIN[base],float)
rows=[]; tot={"occ":[0,np.zeros(3)],"tm":[0,np.zeros(3)]}; Iocc=[];Itm=[]
for pn,shp in sel:
    rho=density(pn)
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(shp,g); V=g.Mass()
    if V<1e-6: continue
    c=g.CentreOfMass(); c=np.array([c.X(),c.Y(),c.Z()]); mi=g.MatrixOfInertia()
    I=np.array([[mi.Value(i,j) for j in (1,2,3)] for i in (1,2,3)])*rho/1e6*1e-6
    m=rho*V/1e6
    Vt,Ft=tris_of(shp); tm=trimesh.Trimesh(Vt,Ft,process=True)  # mm 단위, density 1 → mm^5
    wt=tm.is_watertight
    mt=abs(tm.volume)*rho/1e6
    mp=tm.mass_properties  # 발산정리 적분: watertight 아니어도 계산
    ct=np.array(mp.center_mass)
    It=mp.inertia*rho*1e-6*1e-6   # kg·m² (COM 기준)
    rows.append((pn,rho,V/1e3,m,abs(tm.volume)/1e3,mt,wt,len(tm.faces)))
    tot["occ"][0]+=m; tot["occ"][1]+=m*c; Iocc.append((m,c,I))
    tot["tm"][0]+=mt; tot["tm"][1]+=mt*ct; Itm.append((mt,ct,It))
def comb(L,M,C):
    It=np.zeros((3,3))
    for m,c,I in L:
        r=(c-C)/1000; It+=I+m*(r@r*np.eye(3)-np.outer(r,r))
    return R@It@R.T
res={}
for k,L in (("occ",Iocc),("tm",Itm)):
    M=tot[k][0]; C=tot[k][1]/M; res[k]=(M,R@(C-O)/1000,comb(L,M,C))
with open(OUTMD,"w",encoding="utf-8") as fh:
    fh.write(f"# inertial 3자 비교 — `{LINK}` (링크 프레임 = elbow 관절점, 로봇 좌표)\n\n")
    fh.write("| 방법 | 질량 kg | COM (m) | Ixx · Iyy · Izz (kg·m², COM 기준) | 비고 |\n|---|---|---|---|---|\n")
    lab={"occ":"① OCCT B-rep (FreeCAD와 같은 커널, 재질 가정)","tm":"② trimesh (부품별 메시, 같은 밀도)"}
    for k in ("occ","tm"):
        M,C,I=res[k]; fh.write(f"| {lab[k]} | {M:.4f} | ({C[0]:.4f}, {C[1]:.4f}, {C[2]:.4f}) | {I[0,0]:.5f} · {I[1,1]:.5f} · {I[2,2]:.5f} | |\n")
    d=lambda a,b:(a-b)/a*100
    Mo,Co,Io=res["occ"];Mt,Ct,It=res["tm"]
    fh.write(f"| ③ BOM | – | – | – | **미수령** — 수령 시 질량만 비교 |\n\n차이 ②−①: 질량 {d(Mo,Mt):+.2f} %, COM {np.linalg.norm(Co-Ct)*1000:.2f} mm, Ixx {d(Io[0,0],It[0,0]):+.2f} % · Iyy {d(Io[1,1],It[1,1]):+.2f} % · Izz {d(Io[2,2],It[2,2]):+.2f} %\n\n")
    fh.write("재질 가정: 기어 · 축 · 베어링 · 키 = 강 7.85 g/cm³, 나머지 Al 2.70 g/cm³ (step_to_links.py `STEEL`)\n\n| 부품 | ρ | V_OCCT cm³ | m kg | V_mesh cm³ | m_mesh kg | watertight | 면 수 |\n|---|---|---|---|---|---|---|---|\n")
    for r in rows: fh.write(f"| {r[0]} | {r[1]} | {r[2]:.2f} | {r[3]:.4f} | {r[4]:.2f} | {r[5]:.4f} | {r[6]} | {r[7]} |\n")
print(open(OUTMD,encoding="utf-8").read())

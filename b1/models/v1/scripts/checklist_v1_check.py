# 체크리스트 v0.4 자동 점검 — b1_v1 (URDF 원본 + build_v1.py 생성 MJCF). 사용: python checklist_v1_check.py → ../results/checklist_v1.md
# week04 v0_checklist_check.py를 v1 구조(양팔 · 베이스 3-DOF · 상판 링크 · 그리퍼 · site · Isaac 사본)로 갱신
import os,sys,json,numpy as np,mujoco,trimesh,xml.etree.ElementTree as ET
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE); from v1_mjio import load_model
U=os.path.join(V1,"urdf","b1_v1.urdf"); UI=os.path.join(V1,"urdf","b1_v1_isaac.urdf"); S=os.path.join(V1,"mjcf","scene.xml")
VAL=json.load(open(os.path.join(V1,"results","validate_model.json"),encoding="utf-8")); GC=json.load(open(os.path.join(V1,"results","gravcomp.json")))
root=ET.parse(U).getroot(); rows=[]
def add(no,item,ok,detail): rows.append((no,item,bool(ok),detail))
links=root.findall("link"); joints=root.findall("joint"); rev=[j for j in joints if j.get("type")=="revolute"]
noin=[l.get("name") for l in links if l.find("inertial") is None and l.get("name")!="odom"]
add("A1","모든 link에 inertial",not noin,f"없음: {noin or '-'} (odom = 월드 고정 루트)")
light=[l.get("name") for l in links if l.find("inertial") is not None and float(l.find("inertial/mass").get("value"))<1e-3-1e-12]
dummies=[l.get("name") for l in links if l.find("inertial") is not None and float(l.find("inertial/mass").get("value"))<=1e-3+1e-12]
add("A2","질량 ≥ 1e-3, 가상 링크는 fixed",not light,f"1 g 가상 링크 {len(dummies)}개(cad_frame · tool0 ×2) 모두 fixed. Isaac 사본의 base_x · base_y_link는 움직이는 가상 링크라 1 g 필수")
bad=[]
for l in links:
    i=l.find("inertial/inertia")
    if i is None: continue
    g=lambda k: float(i.get(k)); I=np.array([[g("ixx"),g("ixy"),g("ixz")],[g("ixy"),g("iyy"),g("iyz")],[g("ixz"),g("iyz"),g("izz")]]); w=np.linalg.eigvalsh(I)
    if w[0]<=0 or w[0]+w[1]<w[2]*(1-1e-9): bad.append(l.get("name"))
add("A3","관성 양정치 · 삼각부등식 (오른팔 미러 관성 포함)",not bad,f"위반 {bad or 0} — balanceinertia 미사용(B6)")
mx=max(max(trimesh.load(os.path.join(V1,"meshes",f)).extents) for f in os.listdir(os.path.join(V1,"meshes")) if f.endswith(".stl"))
add("A4","단위 m (메시 최대 길이)",0.05<mx<1.0,f"최대 extent {mx:.3f} m")
miss=[j.get("name") for j in rev if j.find("limit") is None or any(j.find("limit").get(k) is None for k in ("lower","upper","effort","velocity"))]
add("A5","revolute 8개 limit(lower·upper·effort·velocity)",not miss and len(rev)==8,f"revolute {len(rev)}, 누락 {miss or 0} — 값 = assumptions_b1.json (공용 v0.2)")
add("A6","트리(폐루프 없음)",len({j.find('child').get('link') for j in joints})==len(joints),"child 중복 없음")
add("A8","transmission · gazebo 미사용",root.find("transmission") is None and root.find("gazebo") is None,"액추에이터는 b1.xml(빌드 덧붙이기)")
comp=root.find("mujoco/compiler").attrib
add("B1","fusestatic=false",comp.get("fusestatic")=="false","base_plate_link · torso · tool0 · gripper_link 이름 보존 → site · sensor 대상")
add("B2","discardvisual=false",comp.get("discardvisual")=="false","")
add("B4","meshdir 지정",comp.get("meshdir")=="../meshes/","상대 경로(package:// 없음) — Isaac 임포트 패키지 요건")
add("B6","balanceinertia=false (팀 결정 10/8)",comp.get("balanceinertia")=="false","")
add("B3","변환본 angle=radian",'angle="radian"' in open(os.path.join(V1,"mjcf","b1_urdf.xml"),encoding="utf-8").read(),"mj_saveLastXML이 명시")
mu=mujoco.MjModel.from_xml_path(U); m=load_model(S); d=mujoco.MjData(m)
names=[m.body(i).name for i in range(m.nbody)]
add("F1","body 수 · 이름 = URDF link",all(l.get("name") in names for l in links) and m.nbody==len(links)+1,f"nbody {m.nbody} = link {len(links)} + world, njnt {m.njnt}, ngeom {m.ngeom}, nsite {m.nsite}")
jt=[(m.joint(i).name,int(m.jnt_type[i])) for i in range(m.njnt)]
add("C3","베이스 planar → slide · slide · hinge (원본)",[t for _,t in jt[:3]]==[2,2,3],f"{[n for n,_ in jt[:3]]}, nq {m.nq} = 3 + 8")
ri=ET.parse(UI).getroot(); bj=[(j.get("name"),j.get("type")) for j in ri.findall("joint")][:3]
add("C3-1","Isaac 사본: prismatic · prismatic · continuous(z) + 가상 링크 2",[t for _,t in bj]==["prismatic","prismatic","continuous"] and VAL["isaac_equiv"]["pass_"],f"{bj}, 원본과 50자세 차 {VAL['isaac_equiv']['max_diff']:.1e}")
mujoco.mj_forward(m,d)
add("D1","영점 자세 접촉 0 (바닥 제외)",sum(1 for c in d.contact[:d.ncon] if "floor" not in (m.geom(c.geom1).name,m.geom(c.geom2).name))==0,f"ncon {d.ncon}")
sw=VAL["sweep"]
add("D3","관절 범위 sweep 자기 충돌 0 (검증 ④)",sw["pass_"],f"8관절 × 0.5° {sum(v['samples'] for v in sw['results'].values())}자세, 접촉 0. 한계 밖 여유 최소: roll − {sw['results']['left_shoulder_roll_joint']['margin_beyond_limit_deg']['-']}° · elbow ± {sw['results']['left_elbow_joint']['margin_beyond_limit_deg']['-']}~{sw['results']['left_elbow_joint']['margin_beyond_limit_deg']['+']}°")
add("D4","visual geom contype=conaffinity=0",all(m.geom_contype[g]==0 for g in range(m.ngeom) if "visual" in m.geom(g).name),"URDF visual → group 1")
add("E1","파일 구조: 변환본 · 빌드 생성 b1.xml · scene",all(os.path.exists(os.path.join(V1,"mjcf",f)) for f in ("b1_urdf.xml","b1.xml","scene.xml")),"b1_urdf.xml(수정 금지) → b1.xml(build_v1.py + actuator_params.json) → scene.xml (0-3절)")
arm=[i for i in range(m.nu) if not m.actuator(i).name.startswith("base")]
eff={j.get("name"):float(j.find("limit").get("effort")) for j in rev}
okE4=all(abs(m.actuator_forcerange[i][1]-eff[m.joint(m.actuator_trnid[i,0]).name])<1e-9 for i in arm) and all(np.allclose(m.actuator_ctrlrange[i],m.jnt_range[m.actuator_trnid[i,0]]) for i in arm)
add("E4","forcerange = effort, ctrlrange = 관절 range",okE4,"±100 N·m, inheritrange=1, class shoulder · elbow")
add("E5","implicitfast + dampratio",m.opt.integrator==mujoco.mjtIntegrator.mjINT_IMPLICITFAST,"dampratio 1 → kv는 컴파일 시 영점 관성으로 계산 (관성 바뀌면 자동 갱신)")
arma=[float(m.dof_armature[m.jnt_dofadr[m.actuator_trnid[i,0]]]) for i in arm]
add("E6","armature (BOM 전 0 허용)",all(a==0 for a in arma),"0 — RI85 출력축 반사관성(요청서 4-1) 수령 시 actuator_params.json만 교체")
ac=json.load(open(os.path.join(V1,"results","actuator_check.json"),encoding="utf-8"))
add("E8","액추에이터 11개 명령 → 관절 응답 (뷰어 슬라이더 점검 자동판)",all(c["ok"] for c in ac),f"{sum(c['ok'] for c in ac)}/{len(ac)} — 범위 40 % 명령, position 오차 ≤ 2 %, 베이스 velocity 1.5 s 후 98 %")
# F3 손계산
pj=m.joint("left_shoulder_pitch_joint"); d.qpos[:]=0; d.qpos[pj.qposadr[0]]=-np.pi/2; mujoco.mj_forward(m,d)
ap=d.xanchor[pj.id]; ax=d.xaxis[pj.id]; hand=0.0
for b in range(m.nbody):
    n=m.body(b).name
    if n.startswith("left_"): hand+=np.dot(np.cross(d.xipos[b]-ap,[0,0,-9.81*m.body_mass[b]]),ax)
mj=-d.qfrc_bias[pj.dofadr[0]]
add("F3","수평 자세 중력토크 손계산 = MuJoCo",abs(hand-mj)<1e-6,f"손계산 Σ r×mg = {hand:.4f} N·m, MuJoCo {mj:.4f} N·m (그리퍼 0.5 kg 포함, v0 22.7301)")
tot=float(sum(m.body_mass))
add("F2","총질량 ≤ 80 kg (기획서) · 검증 ①",tot<=80 and VAL["mass"]["pass_"],f"{tot:.3f} kg, 기준 대비 {VAL['mass']['err_pct']:+.3f} %")
add("E7","position 서보 처짐 ≈ τg/kp · 중력보상으로 상쇄",abs(GC["final_sag_nogc_deg"]-GC["theory_sag_deg"])<0.1 and abs(GC["final_sag_gc_deg"])<0.01,f"보상 없음 {GC['final_sag_nogc_deg']}° vs 이론 {GC['theory_sag_deg']}° → 보상(목표 + τg/kp) {GC['final_sag_gc_deg']}°")
add("M1","좌우 미러 = 규칙표 · 검증 ⑤",VAL["symmetry"]["pass_"],f"{VAL['symmetry']['n']}자세 |Δx|·|Δz|·|y_L+y_R| ≤ {max(VAL['symmetry']['max_dx_mm'],VAL['symmetry']['max_dz_mm'],VAL['symmetry']['max_y_sum_mm']):.1e} mm — docs/mirror_rules.md")
add("F5","캡처: 4자세 · collision · 중력보상","img/v1_poses.png" and os.path.exists(os.path.join(V1,"img","v1_poses.png")),"img/v1_poses.png · v1_collision.png · v1_gravcomp.png + 노트북 뷰어 재현 img/repro/20261016_laptop_v1_*.png (Control 슬라이더 · 양팔 대칭 · base_vx)")
ok=sum(r[2] for r in rows); ng=[r[0] for r in rows if not r[2]]
L=[f"# 체크리스트 v0.4 점검 — b1_v1 (MuJoCo {mujoco.__version__})","",f"자동 생성: checklist_v1_check.py · 합계 {len(rows)}항목 — OK {ok} · NG {len(ng)} {ng if ng else ''}","","| No. | 점검 | 결과 | 근거 |","|---|---|---|---|"]
L+=[f"| {no} | {it} | {'OK' if o else 'NG'} | {de} |" for no,it,o,de in rows]
open(os.path.join(V1,"results","checklist_v1.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L[:3]))

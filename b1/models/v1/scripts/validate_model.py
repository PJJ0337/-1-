# v1 모델 검증 5종 (week05 커리큘럼) + 보조 검증 — 결과: ../results/validate_model.{json,md}
#   ① 질량 합계  ② 정지 유지(+ 중력보상)  ③ 중력 토크 vs 구동기  ④ 관절 범위 sweep  ⑤ 좌우 대칭
#   보조: Isaac용 URDF(prismatic · prismatic · revolute) = 원본(planar) 동등성
# 담당(커리큘럼): ④ · ⑤ 박종진, ① · ② · ③ 함태훈 — 이 스크립트는 B1 공용 기준 계산(함태훈 결과와 대조용)
# 사용: python validate_model.py [scene.xml]
import os,sys,json,itertools,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); MODELS=os.path.dirname(V1)
sys.path.insert(0,HERE); from v1_mjio import load_model
SCENE=sys.argv[1] if len(sys.argv)>1 else os.path.join(V1,"mjcf","scene.xml")
A=json.load(open(os.path.join(MODELS,"assumptions_b1.json"),encoding="utf-8"))
m=load_model(SCENE); d=mujoco.MjData(m)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; SIDES=("left","right")
qa=lambda n: m.jnt_qposadr[m.joint(n).id]; da=lambda n: m.jnt_dofadr[m.joint(n).id]
R={"model":dict(nq=m.nq,nv=m.nv,nu=m.nu,nbody=m.nbody,nsensor=m.nsensor,mujoco=mujoco.__version__)}
RATED=46.0   # PDR 판정 기준: 정격 ≥ 46 N·m (팀장 10/16, 강 상한 38.4 × 1.2) — RI85 정격 수령 시 교체
PEAK=A["actuator"]["effort_peak"]

# ① 질량 합계 — BOM 미수령 → 기준 = B1 공용 가정값 표 + 함태훈 CAD 링크 질량(밀도 3종)
b=A["base"]; t=A["torso"]; g=A["gripper"]
INER=json.load(open(os.path.join(os.path.dirname(MODELS),"..","members/park-jongjin/week04/results/link_inertials_taehoon_in_b1frame.json"),encoding="utf-8"))
ref={"base_link":b["mass_lower"],"base_plate_link":b["mass_plate"],"torso_link":t["box_mass"]+INER["torso_link"]["mass"]}
for s in SIDES:
    for a in ARM: ref[f"{s}_{a}_link"]=INER[f"left_{a}_link"]["mass"]          # 공칭: 오른팔 = 왼팔 거울
    ref[f"{s}_gripper_link"]=g["mass"]
rows=[];
for n,v in ref.items():
    mm=float(m.body_mass[m.body(n).id]); rows.append(dict(link=n,model=round(mm,4),ref=round(v,4),err_pct=round((mm-v)/v*100,3)))
tot=float(m.body_subtreemass[0]); reft=sum(ref.values()); dummy=tot-sum(r["model"] for r in rows)
R["mass"]=dict(total=round(tot,3),ref_total=round(reft,3),err_pct=round((tot-reft)/reft*100,3),dummy_kg=round(dummy,4),links=rows,
    th_cad_right_arm=round(sum(INER[f"right_{a}_link"]["mass"] for a in ARM),3),
    pass_=bool(abs(tot-reft)/reft<0.05 and all(abs(r["err_pct"])<5 for r in rows)))

# ② 정지 유지: 목표 = 현재 각도, 5 s. (a) 영점 (b) 양팔 앞으로 수평 — 중력보상 없음 / 있음(ctrl = q* + τg/kp 피드포워드)
def hold(pose,gc,T=5.0):
    mujoco.mj_resetData(m,d)
    for j,v in pose.items(): d.qpos[qa(j)]=v
    mujoco.mj_forward(m,d); q0=d.qpos.copy(); err=[]
    kp={}
    for s in SIDES:
        for a in ARM: kp[f"{s}_{a}"]=float(m.actuator_gainprm[m.actuator(f"{s}_{a}").id][0])
    for k in range(int(T/m.opt.timestep)):
        for s in SIDES:
            for a in ARM:
                j=f"{s}_{a}_joint"; ai=m.actuator(f"{s}_{a}").id
                # 중력보상: qfrc_bias(qvel≈0 → 중력항)를 position 목표 오프셋 τg/kp로 피드포워드
                d.ctrl[ai]=q0[qa(j)]+(d.qfrc_bias[da(j)]/kp[f"{s}_{a}"] if gc else 0.0)
        mujoco.mj_step(m,d)
        e=max(abs(d.qpos[qa(f"{s}_{a}_joint")]-q0[qa(f"{s}_{a}_joint")]) for s in SIDES for a in ARM); err.append(e)
    err=np.array(err); last=err[int(len(err)*0.8):]
    return dict(max_err_deg=round(float(np.rad2deg(err.max())),4),final_err_deg=round(float(np.rad2deg(err[-1])),4),
                jitter_deg=round(float(np.rad2deg(last.max()-last.min())),5),finite=bool(np.isfinite(d.qpos).all()),
                ncon=int(sum(1 for c in d.contact[:d.ncon] if "floor" not in (m.geom(c.geom1).name,m.geom(c.geom2).name))),
                pass_=bool(np.rad2deg(err[-1])<=0.5 and np.isfinite(d.qpos).all()))
FWD={f"{s}_shoulder_pitch_joint":-np.pi/2 for s in SIDES}
R["hold"]={"zero_nogc":hold({},False),"forward_nogc":hold(FWD,False),"zero_gc":hold({},True),"forward_gc":hold(FWD,True)}
R["hold"]["pass_"]=R["hold"]["zero_nogc"]["pass_"] and R["hold"]["forward_gc"]["pass_"]

# ③ 중력 토크 vs 구동기: 왼팔 4관절 격자(15°)에서 관절별 |τg| 최대 자세, 페이로드 0 / 3 kg(ee site)
ee=m.site("left_ee").id; ebody=m.site_bodyid[ee]
def tau_g(q,payload):
    d.qpos[:]=0; d.qvel[:]=0
    for a,v in zip(ARM,q): d.qpos[qa(f"left_{a}_joint")]=v
    mujoco.mj_forward(m,d); tau=np.array([d.qfrc_bias[da(f"left_{a}_joint")] for a in ARM])
    if payload:
        jac=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jac,None,ee)
        tau=tau+np.array([-(jac[:,da(f"left_{a}_joint")]@np.array([0,0,-9.81*payload])) for a in ARM])
    return tau
grid=[np.deg2rad(np.arange(np.ceil(np.rad2deg(lo)/15)*15,np.rad2deg(hi)+1e-6,15)) for lo,hi in (A["limits"][a] for a in ARM)]   # 0 · ±90° 포함 15° 격자
grav={}
for pl in (0.0,3.0):
    best=[(0,None)]*4
    for q in itertools.product(*grid):
        tau=np.abs(tau_g(q,pl))
        for i in range(4):
            if tau[i]>best[i][0]: best[i]=(float(tau[i]),[round(float(np.rad2deg(x)),0) for x in q])
    grav[f"{pl:g}kg"]={a:dict(tau=round(best[i][0],2),pose_deg=best[i][1],margin_rated_pct=round((RATED-best[i][0])/best[i][0]*100,1),
                             margin_peak_pct=round((PEAK-best[i][0])/best[i][0]*100,1)) for i,a in enumerate(ARM)}
R["gravity"]=dict(rated_criterion=RATED,peak=PEAK,results=grav,horizontal_0kg=round(float(abs(tau_g([-np.pi/2,0,0,0],0)[0])),3),
                  horizontal_3kg=round(float(abs(tau_g([-np.pi/2,0,0,0],3)[0])),3),
                  pass_=all(v["tau"]*1.2<=RATED for v in grav["3kg"].values()))

# ④ 관절 범위 sweep: 관절 하나씩 하한 → 상한(0.5° 간격), 나머지 0, 바닥 제외 접촉 기록 (양팔)
def contacts():
    mujoco.mj_forward(m,d); out=[]
    for c in d.contact[:d.ncon]:
        g1,g2=m.geom(c.geom1).name,m.geom(c.geom2).name
        if "floor" in (g1,g2): continue
        out.append((m.body(m.geom_bodyid[c.geom1]).name,m.body(m.geom_bodyid[c.geom2]).name,round(float(c.dist),4)))
    return out
sweep={}
for s in SIDES:
    for a in ARM:
        j=f"{s}_{a}_joint"; lo,hi=m.jnt_range[m.joint(j).id]; hits=[]; n=0
        for q in np.arange(lo,hi+1e-9,np.deg2rad(0.5)):
            d.qpos[:]=0; d.qpos[qa(j)]=q; cs=contacts(); n+=1
            if cs: hits.append((round(float(np.rad2deg(q)),1),cs))
        # 범위 밖 여유: 한계에서 첫 접촉까지 (같은 방향으로 계속)
        margin={}
        for sgn,lim in ((-1,lo),(1,hi)):
            mg=None
            for k in range(1,181):
                q=lim+sgn*np.deg2rad(k)
                if abs(q)>np.pi: break
                d.qpos[:]=0; d.qpos[qa(j)]=q
                if contacts(): mg=k; break
            margin["-" if sgn<0 else "+"]=mg
        sweep[j]=dict(range_deg=[round(float(np.rad2deg(lo)),1),round(float(np.rad2deg(hi)),1)],samples=n,hits=len(hits),first_hit=hits[0] if hits else None,margin_beyond_limit_deg=margin)
R["sweep"]=dict(results=sweep,pass_=all(v["hits"]==0 for v in sweep.values()),excluded_pairs="MuJoCo 기본: 부모-자식 body 접촉 제외 (추가 exclude 없음)")

# ⑤ 좌우 대칭: 같은 관절각(안 A 거울 → 같은 명령) 무작위 500자세 + 대표 4자세
rng=np.random.default_rng(5); lims=[A["limits"][a] for a in ARM]
def ee_pair(q):
    d.qpos[:]=0
    for a,v in zip(ARM,q): d.qpos[qa(f"left_{a}_joint")]=v; d.qpos[qa(f"right_{a}_joint")]=v
    mujoco.mj_forward(m,d); return d.site_xpos[m.site("left_ee").id].copy(),d.site_xpos[m.site("right_ee").id].copy()
dx=[];dz=[];dy=[]
qs=[np.zeros(4),np.array([-np.pi/2,0,0,0]),np.array([0,1.2,0,0]),np.array([-0.6,0.3,0.5,1.4])]+[np.array([rng.uniform(lo,hi) for lo,hi in lims]) for _ in range(500)]
for q in qs:
    l,r=ee_pair(q); dx.append(abs(l[0]-r[0])); dz.append(abs(l[2]-r[2])); dy.append(abs(l[1]+r[1]))
l0,r0=ee_pair(np.zeros(4))
R["symmetry"]=dict(n=len(qs),max_dx_mm=round(max(dx)*1000,6),max_dz_mm=round(max(dz)*1000,6),max_y_sum_mm=round(max(dy)*1000,6),
                   zero_left_ee=[round(float(x),4) for x in l0],zero_right_ee=[round(float(x),4) for x in r0],
                   pass_=bool(max(dx)<1e-3 and max(dz)<1e-3 and max(dy)<1e-3))

# 보조: Isaac용 URDF 동등성 (질량 · ee 위치 · 중력토크)
mi=mujoco.MjModel.from_xml_path(os.path.join(V1,"urdf","b1_v1_isaac.urdf")); di=mujoco.MjData(mi)
mp=mujoco.MjModel.from_xml_path(os.path.join(V1,"urdf","b1_v1.urdf")); dp=mujoco.MjData(mp)
eq=[]
for q in qs[:50]:
    for mm_,dd_ in ((mi,di),(mp,dp)):
        dd_.qpos[:]=0
        for a,v in zip(ARM,q): dd_.qpos[mm_.jnt_qposadr[mm_.joint(f"left_{a}_joint").id]]=v
        mujoco.mj_forward(mm_,dd_)
    eq.append(max(np.abs(di.xpos[mi.body("left_tool0").id]-dp.xpos[mp.body("left_tool0").id]).max(),
              abs(di.qfrc_bias[mi.jnt_dofadr[mi.joint("left_shoulder_pitch_joint").id]]-dp.qfrc_bias[mp.jnt_dofadr[mp.joint("left_shoulder_pitch_joint").id]])))
R["isaac_equiv"]=dict(isaac_nq=mi.nq,isaac_joints=[mi.joint(i).name for i in range(3)],mass_planar=round(float(sum(mp.body_mass)),4),
                      mass_isaac=round(float(sum(mi.body_mass)),4),max_diff=float(max(eq)),pass_=bool(max(eq)<1e-9))

json.dump(R,open(os.path.join(V1,"results","validate_model.json"),"w"),indent=1,ensure_ascii=False)
ok=lambda b:"**통과**" if b else "**미통과**"
L=["# v1 모델 검증 5종 결과 (자동 생성 — validate_model.py)","",f"모델: `mjcf/scene.xml` — nq {m.nq} · nv {m.nv} · nu {m.nu} · nbody {m.nbody} · 센서 {m.nsensor}, MuJoCo {mujoco.__version__}","",
   "| 검증 | 통과 기준 | 결과 | 판정 |","|---|---|---|---|",
   f'| ① 질량 합계 | 전체 · 링크별 ±5 % (기준 = 공용 가정값 + 함태훈 CAD 질량, BOM 대기) | 모델 {R["mass"]["total"]} kg vs 기준 {R["mass"]["ref_total"]} kg ({R["mass"]["err_pct"]:+.3f} %), 링크별 최대 {max(abs(r["err_pct"]) for r in rows):.3f} % | {ok(R["mass"]["pass_"])} |',
   f'| ② 정지 유지 | 5 s, 관절 오차 ≤ 0.5°, 발산 · 떨림 없음 | 영점 {R["hold"]["zero_nogc"]["final_err_deg"]}°, 앞으로 수평: 보상 없음 {R["hold"]["forward_nogc"]["final_err_deg"]}° → **중력보상 {R["hold"]["forward_gc"]["final_err_deg"]}°** (떨림 {R["hold"]["forward_gc"]["jitter_deg"]}°) | {ok(R["hold"]["pass_"])} (수평은 중력보상 필요) |',
   f'| ③ 중력 토크 vs 구동기 | 관절별 최악 |τg| × 1.2 ≤ 정격 (판정 기준 {RATED:g} N·m, 정격 미수령) | 3 kg: pitch {grav["3kg"]["shoulder_pitch"]["tau"]} · roll {grav["3kg"]["shoulder_roll"]["tau"]} · yaw {grav["3kg"]["shoulder_yaw"]["tau"]} · elbow {grav["3kg"]["elbow"]["tau"]} N·m | {ok(R["gravity"]["pass_"])} (정격 가정) |',
   f'| ④ 관절 범위 sweep | 범위 안 자기 충돌 0건 | {sum(v["samples"] for v in sweep.values())}자세(8관절 × 0.5°), 접촉 {sum(v["hits"] for v in sweep.values())}건 | {ok(R["sweep"]["pass_"])} |',
   f'| ⑤ 좌우 대칭 | y 부호만 다르고 x · z 차 ≤ 1 mm | {R["symmetry"]["n"]}자세 최대 |Δx| {R["symmetry"]["max_dx_mm"]:.2e} · |Δz| {R["symmetry"]["max_dz_mm"]:.2e} · |y_L + y_R| {R["symmetry"]["max_y_sum_mm"]:.2e} mm | {ok(R["symmetry"]["pass_"])} |',
   f'| 보조 · Isaac용 URDF | planar ↔ prismatic · prismatic · revolute 동등 | 질량 {R["isaac_equiv"]["mass_planar"]} vs {R["isaac_equiv"]["mass_isaac"]} kg(가상 링크 2 g), 50자세 최대 차 {R["isaac_equiv"]["max_diff"]:.1e} | {ok(R["isaac_equiv"]["pass_"])} |',
   "","## ① 링크별 질량","","| 링크 | 모델 kg | 기준 kg | 오차 % |","|---|---|---|---|"]+[f'| {r["link"]} | {r["model"]} | {r["ref"]} | {r["err_pct"]} |' for r in rows]+[
   "",f'- 더미 프레임(odom · cad_frame · tool0 등) {R["mass"]["dummy_kg"]} kg 포함. 오른팔 기준 = 왼팔 거울(공칭). 함태훈 CAD 오른팔 {R["mass"]["th_cad_right_arm"]} kg과 차 {R["mass"]["th_cad_right_arm"]-sum(ref[f"right_{a}_link"] for a in ARM):+.3f} kg (0.4 %)',
   "","## ② 정지 유지 (kp 300, dampratio 1)","","| 자세 | 중력보상 | 최대 오차 ° | 5 s 후 오차 ° | 마지막 1 s 떨림 ° | 접촉 |","|---|---|---|---|---|---|"]+[
   f'| {k.split("_")[0]} | {"있음" if k.endswith("gc") and not k.endswith("nogc") else "없음"} | {v["max_err_deg"]} | {v["final_err_deg"]} | {v["jitter_deg"]} | {v["ncon"]} |' for k,v in R["hold"].items() if k!="pass_"]+[
   "","- 중력보상 = 위치 목표에 τg / kp 오프셋 (qfrc_bias, qvel ≈ 0에서 중력항). 토크 명령이 아니라 position 서보 목표만 바꾸므로 forcerange 제한 그대로",
   "",f"## ③ 관절별 최악 중력 토크 (왼팔 15° 격자 전수(0° 기준), 판정 기준 정격 {RATED:g} N·m · 피크 {PEAK:g} N·m 가정)","",
   "| 관절 | 0 kg τ [N·m] (자세 °) | 3 kg τ [N·m] (자세 °) | 3 kg 정격 여유 % | 3 kg 피크 여유 % |","|---|---|---|---|---|"]+[
   f'| {a} | {grav["0kg"][a]["tau"]} ({grav["0kg"][a]["pose_deg"]}) | {grav["3kg"][a]["tau"]} ({grav["3kg"][a]["pose_deg"]}) | {grav["3kg"][a]["margin_rated_pct"]} | {grav["3kg"][a]["margin_peak_pct"]} |' for a in ARM]+[
   "",f'- 팔 앞으로 수평(pitch −90°) 기준: 0 kg {R["gravity"]["horizontal_0kg"]} · 3 kg {R["gravity"]["horizontal_3kg"]} N·m — v0(그리퍼 없음, 페이로드 = 플랜지) 22.73 / 37.30 대비 증가분 = 그리퍼 box 0.5 kg(S14 가정) + 페이로드 작용점이 플랜지 → ee(+0.10 m)로 이동. 자세 = (pitch, roll, yaw, elbow)',
   "","## ④ 관절 범위 sweep","","| joint | 범위 ° | 샘플 | 범위 안 접촉 | 한계 밖 첫 접촉까지 여유 ° (−/+) |","|---|---|---|---|---|"]+[
   f'| {j} | {v["range_deg"][0]} ~ {v["range_deg"][1]} | {v["samples"]} | {v["hits"]} | {v["margin_beyond_limit_deg"]["-"]} / {v["margin_beyond_limit_deg"]["+"]} |' for j,v in sweep.items()]+[
   "","- 여유 None = 한계 밖으로 180°(또는 ±π)까지 접촉 없음. 충돌 판정 = collision 메시(convex hull) · box, 부모-자식 접촉은 MuJoCo 기본 제외, 추가 `exclude` · `contype` 조치 없음",
   "","## ⑤ 좌우 대칭",f'- 영점 left_ee {R["symmetry"]["zero_left_ee"]} · right_ee {R["symmetry"]["zero_right_ee"]} m','- 미러 규칙표: [../docs/mirror_rules.md](../docs/mirror_rules.md)']
open(os.path.join(V1,"results","validate_model.md"),"w",encoding="utf-8").write("\n".join(L)+"\n")
print("\n".join(L[:13]))

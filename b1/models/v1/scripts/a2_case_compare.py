# A2 구동기 case 비교 (week06) — armature 0(week05) · cad_ri85(8개 RI85) · a2_doc(어깨 TQ + 팔꿈치 RI85)
#   같은 kp 300 · dampratio 1(kv는 컴파일 시 armature 포함 관성으로 다시 계산), 중력보상 B 켠 상태에서 관절별 0.2 rad 계단 응답
#   → 오버슈트 · 2 % 정착시간 · 고유진동수 ω_n = √(kp / I_eff), 그리고 팔 앞으로 수평 정지 유지 5 s 오차
#   결과: results/a2_case_compare.{json,md}
import os,sys,json,numpy as np,mujoco,xml.etree.ElementTree as ET
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); sys.path.insert(0,HERE); from v1_mjio import _assets
SCENE=os.path.join(V1,"mjcf","scene.xml"); A2=json.load(open(os.path.join(V1,"inputs","a2_actuators.json"),encoding="utf-8"))
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
CASES={"none (week05, armature 0)":{"shoulder":(0.0,100.0),"elbow":(0.0,100.0)}}
for k,c in A2["cases"].items(): CASES[k]={cl:(v["armature"],v["peak"]) for cl,v in c["classes"].items()}
def model(case):
    a=_assets(SCENE); r=ET.fromstring(a["b1.xml"].decode("utf-8"))
    for dd in r.iter("default"):
        cl=dd.get("class")
        if cl in case:
            arm,fr=case[cl]; j=dd.find("joint"); p=dd.find("position"); j.set("armature",f"{arm}"); p.set("forcerange",f"{-fr} {fr}")
            if j.get("actuatorfrcrange"): j.set("actuatorfrcrange",f"{-fr} {fr}")
    a["b1.xml"]=ET.tostring(r,encoding="unicode").encode("utf-8")
    return mujoco.MjModel.from_xml_string(open(SCENE,encoding="utf-8").read(),assets=a)
out={}
for name,case in CASES.items():
    m=model(case); d=mujoco.MjData(m); res={}
    for a in ARM:
        j=m.joint(f"left_{a}_joint"); ai=m.actuator(f"left_{a}").id; qa=m.jnt_qposadr[j.id]; da=m.jnt_dofadr[j.id]
        mujoco.mj_resetData(m,d); mujoco.mj_forward(m,d); M=np.zeros((m.nv,m.nv)); mujoco.mj_fullM(m,d,M)
        step=0.2; d.ctrl[ai]=step; y=[]
        for k in range(1500): mujoco.mj_step(m,d); y.append(d.qpos[qa])
        y=np.array(y); t=np.arange(1,1501)*m.opt.timestep; band=np.abs(y-step)>0.02*step
        ts=float(t[np.where(band)[0][-1]]) if band.any() else 0.0
        res[a]=dict(armature=float(m.dof_armature[da]),kv=round(float(m.actuator_biasprm[ai][2]*-1),3),I_eff_hold=round(float(M[da,da]),4),
                    wn=round(float(np.sqrt(300/M[da,da])),1),overshoot_pct=round(float(max(0,(y.max()-step)/step*100)),2),settle_s=round(ts,3))
    # 앞으로 수평 정지 유지 (양팔 pitch −90°, 5 s)
    mujoco.mj_resetData(m,d)
    for s in ("left","right"):
        d.qpos[m.jnt_qposadr[m.joint(f"{s}_shoulder_pitch_joint").id]]=-np.pi/2; d.ctrl[m.actuator(f"{s}_shoulder_pitch").id]=-np.pi/2
    for k in range(2500): mujoco.mj_step(m,d)
    res["hold_forward_err_deg"]=round(float(np.rad2deg(abs(d.qpos[m.jnt_qposadr[m.joint("left_shoulder_pitch_joint").id]]+np.pi/2))),4)
    out[name]=res
json.dump(out,open(os.path.join(V1,"results","a2_case_compare.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False)
L=["# A2 구동기 case 비교 (자동 생성 — a2_case_compare.py)","","kp 300 · dampratio 1(kv 재계산), 중력보상 B on, 영점에서 왼팔 관절별 0.2 rad 계단 3 s. I_eff = 영점 질량행렬 대각(다른 관절 서보 고정 근사)","",
   "| case | 관절 | armature [kg·m²] | kv | I_eff [kg·m²] | ω_n [rad/s] | 오버슈트 % | 2 % 정착 [s] |","|---|---|---|---|---|---|---|---|"]
for n,r in out.items():
    for a in ARM: v=r[a]; L.append(f'| {n} | {a} | {v["armature"]} | {v["kv"]} | {v["I_eff_hold"]} | {v["wn"]} | {v["overshoot_pct"]} | {v["settle_s"]} |')
L+=["",*(f'- {n}: 팔 앞으로 수평 5 s 유지 오차 {r["hold_forward_err_deg"]}°' for n,r in out.items()),
    "- armature가 가장 가벼운 관절(yaw · elbow)의 I_eff를 몇 배로 키움 → ω_n이 크게 내려가고 정착이 느려짐(함태훈 armature_study.md 예측과 같은 방향). 로터만 값이라 실제는 더 큼(기어열 TBD)",
    "- kp · kv 재튜닝(함태훈 6주차 과제)은 a2_doc 기준으로 할 것을 제안 — 어깨 TQ/RI85 확정 시 case만 바꾸고 다시 빌드"]
open(os.path.join(V1,"results","a2_case_compare.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

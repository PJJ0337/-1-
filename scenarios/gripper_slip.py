# week08 박종진 — 임시 2지 그리퍼 파지 유지 점검: 마찰 × 파지력 × 물체 질량 → 들어 올린 뒤 2 s 유지 중 미끄러짐
#   (커리큘럼 8주차 월 실습. 함태훈 그리퍼 · 접촉 스윕(solref · condim)이 오면 그 표가 기준, 이 표는 pick_env 가정값 확인용)
#   손바닥 = 수직 slide 관절 + 강한 position 서보(로봇 팔 대신, 접촉 · 마찰만 보려고. mocap으로 끌면 속도 0이라 마찰이 안 걸림), 손가락 · 접촉 파라미터 = pick_env.GRIPPER · CONTACT 그대로
#   이론: 들기 유지 조건 2 μ F ≥ m g (손가락 2개 × 마찰) → F_min = m g / (2 μ)
#   출력: results/gripper_slip.{json,md}     실행: python scenarios/gripper_slip.py   (~10 s)
import os,sys,json,itertools
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from pick_env import GRIPPER,CONTACT,OBJ,OPTION,_f

def scene(mu,F,mass,kind,condim=CONTACT["condim"],opt=OPTION):
    G,C=GRIPPER,CONTACT; hx,hy,hz=G["finger_half"]; fr=f"{mu} 0.005 0.0001"
    cc=f'condim="{condim}" solref="{_f(C["solref"])}" solimp="{_f(C["solimp"])}"'
    fing="".join(f'<body name="f{n}" pos="0 0 0"><inertial pos="{s*hx} 0 {-hz}" mass="{G["finger_mass"]}" diaginertia="2e-5 2e-5 5e-6"/>'
                 f'<joint name="j{n}" type="slide" axis="{s} 0 0" range="0 {G["stroke"]}" damping="{G["damping"]}" armature="0.01"/>'
                 f'<geom type="box" size="{hx} {hy} {hz}" pos="{s*hx} 0 {-hz}" friction="{fr}" {cc}/></body>' for n,s in (("l",1),("r",-1)))
    og=(f'<geom type="box" size="{OBJ["box_half"]} {OBJ["box_half"]} {OBJ["box_half"]}"' if kind=="box" else
        f'<geom type="cylinder" size="{OBJ["cyl_r"]} {OBJ["cyl_half"]}"')+f' mass="{mass}" friction="{fr}" {cc}/>'
    h=OBJ["box_half"] if kind=="box" else OBJ["cyl_half"]; gz=2*h-0.002+G["grasp_off"]-0.002   # 손바닥 아래면 높이(파지 중심 = 물체 중심 근처)
    gz=h+0.002+G["grasp_off"]
    xml=f'''<mujoco><option integrator="implicitfast" {opt}/>
<worldbody><geom name="tbl" type="box" size="0.3 0.3 0.05" pos="0 0 -0.05" friction="{_f(C["table_friction"])}" {cc}/>
<body name="palm" pos="0 0 {gz}"><joint name="lift" type="slide" axis="0 0 1" damping="200"/><geom type="box" size="0.04 0.04 0.05" pos="0 0 0.05" mass="1" contype="0" conaffinity="0"/>{fing}</body>
<body name="obj" pos="0 0 {h}"><freejoint/>{og}</body></worldbody>
<contact><exclude body1="fl" body2="fr"/></contact>
<equality><joint joint1="jr" joint2="jl" polycoef="0 1 0 0 0" solref="0.005 1"/></equality>
<actuator><position joint="lift" kp="20000" kv="400"/>{"".join(f'<position joint="j{n}" kp="{G["kp"]}" ctrlrange="0 {G["stroke"]}" forcerange="{-F} {F}"/>' for n in "lr")}</actuator></mujoco>'''
    return mujoco.MjModel.from_xml_string(xml)

def trial(mu,F,mass,kind,condim=CONTACT["condim"],opt=OPTION):
    m=scene(mu,F,mass,kind,condim,opt); d=mujoco.MjData(m); jl=m.jnt_qposadr[m.joint("jl").id]; jr=m.jnt_qposadr[m.joint("jr").id]
    d.qpos[jl]=d.qpos[jr]=GRIPPER["stroke"]; d.ctrl[1:]=GRIPPER["stroke"]; mujoco.mj_forward(m,d)
    ob=m.body("obj").id; pb=m.body("palm").id; h0=d.xpos[ob,2]
    def run(T,lift=0.0,t_lift=0.0):
        n=int(T/m.opt.timestep); zs=d.ctrl[0]
        for i in range(n):
            if t_lift>0: s=min(1.0,(i+1)*m.opt.timestep/t_lift); d.ctrl[0]=zs+lift*(10*s**3-15*s**4+6*s**5)
            mujoco.mj_step(m,d)
    run(0.3); d.ctrl[1:]=0.0; run(0.6)                   # 닫기(손가락당 힘 상한 F까지)
    rel0=d.xpos[ob,2]-d.xpos[pb,2]; w=float(d.qpos[jl])
    run(0.6,lift=0.10,t_lift=0.6); rel1=d.xpos[ob,2]-d.xpos[pb,2]
    run(2.0); rel2=d.xpos[ob,2]-d.xpos[pb,2]       # 2 s 유지
    slip_lift=(rel0-rel1)*1e3; slip_hold=(rel1-rel2)*1e3; dropped=d.xpos[ob,2]<h0+0.02
    pen=-min([d.contact[i].dist for i in range(d.ncon)]+[0.0])*1e3
    return dict(mu=mu,F=F,mass=mass,kind=kind,condim=condim,width_mm=round(2*w*1e3,1),slip_lift_mm=round(slip_lift,2),slip_hold_mm=round(slip_hold,2),
                dropped=bool(dropped),held=bool((not dropped) and slip_lift+slip_hold<5.0),pen_mm=round(pen,2),F_min=round(mass*9.81/(2*mu),1))

if __name__=="__main__":
    rows=[trial(mu,F,ms,k) for mu,F,ms,k in itertools.product((0.3,0.6,1.0),(10.0,20.0,40.0),(0.2,1.0,3.0),("box","cyl"))]
    rows+= [trial(1.0,40.0,3.0,k,c) for k in ("box","cyl") for c in (3,6)]
    # 마찰 원뿔 비교: MuJoCo 기본(pyramidal, impratio 1) — 같은 54조합
    PYR='timestep="0.0005" cone="pyramidal" impratio="1"'
    pyr=[trial(mu,F,ms,k,opt=PYR) for mu,F,ms,k in itertools.product((0.3,0.6,1.0),(10.0,20.0,40.0),(0.2,1.0,3.0),("box","cyl"))]
    pyr_ok=sum(r["held"]==(r["F"]>=r["F_min"]) for r in pyr); pyr_worst=max(r["slip_lift_mm"]+r["slip_hold_mm"] for r in pyr if r["F"]>=1.5*r["F_min"])
    el_worst=max(r["slip_lift_mm"]+r["slip_hold_mm"] for r in rows if r["condim"]==CONTACT["condim"] and r["F"]>=1.5*r["F_min"])
    ok_th=sum(1 for r in rows if r["condim"]==CONTACT["condim"] and r["held"]==(r["F"]>=r["F_min"]))
    n4=sum(1 for r in rows if r["condim"]==CONTACT["condim"])
    out=os.path.join(os.path.dirname(HERE),"scenarios","results"); os.makedirs(out,exist_ok=True)
    json.dump(dict(gripper=GRIPPER,contact=CONTACT,option=OPTION,rows=rows,theory_match=[ok_th,n4],pyramidal=dict(rows=pyr,theory_match=[pyr_ok,len(pyr)],worst_slip_mm=pyr_worst),elliptic_worst_slip_mm=el_worst),open(os.path.join(out,"gripper_slip.json"),"w"),indent=1,ensure_ascii=False)
    L=["# 임시 2지 그리퍼 파지 유지 — 마찰 × 파지력 × 질량 (자동 생성 — gripper_slip.py)","",
       f"가정값: 손가락당 힘 상한 F(표), 손가락 {2*GRIPPER['finger_half'][2]*1e3:.0f} mm, stroke {GRIPPER['stroke']*1e3:.0f} mm/손가락, condim {CONTACT['condim']}, solref {CONTACT['solref']}, solimp {CONTACT['solimp']}. "
       f"option {OPTION}. 손바닥 = 수직 slide 서보, 닫기 0.6 s → 0.10 m 들기(0.6 s, 5차) → 2 s 유지. 유지 판정 = 낙하 없음 · 상대 미끄러짐 < 5 mm. 이론 F_min = m g / (2 μ)","",
       "| μ | F [N] | 질량 [kg] | 물체 | F_min 이론 [N] | 들기 중 미끄러짐 [mm] | 유지 중 [mm] | 결과 |","|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["condim"]!=CONTACT["condim"]: continue
        L.append(f"| {r['mu']} | {r['F']:.0f} | {r['mass']} | {r['kind']} | {r['F_min']} | {r['slip_lift_mm']} | {r['slip_hold_mm']} | {'유지' if r['held'] else ('낙하' if r['dropped'] else '미끄러짐')} |")
    L+=["",f"- 이론(F ≥ F_min ⇔ 유지)과 일치: **{ok_th}/{n4}**, F ≥ 1.5 F_min에서 최대 미끄러짐 {el_worst:.2f} mm",
        "","## 마찰 원뿔 설정 비교 (같은 54조합)","","| option | 이론 일치 | F ≥ 1.5 F_min 최대 미끄러짐 [mm] |","|---|---|---|",
        f"| {OPTION} (채택) | {ok_th}/{n4} | {el_worst:.2f} |",f"| {PYR} (MuJoCo 기본) | {pyr_ok}/{len(pyr)} | {pyr_worst:.2f} |","",
        "- 기본(pyramidal)은 힘이 충분해도 천천히 흘러내림(soft contact 크리프) → 파지 scene은 elliptic + impratio 10 (MuJoCo 문서 권장: 미끄러짐 방지 · 조작 작업)","","## condim 비교 (μ 1.0 · F 40 N · 3 kg)","","| 물체 | condim | 들기 [mm] | 유지 [mm] | 결과 |","|---|---|---|---|---|"]
    for r in rows:
        if r["mu"]==1.0 and r["F"]==40.0 and r["mass"]==3.0: L.append(f"| {r['kind']} | {r['condim']} | {r['slip_lift_mm']} | {r['slip_hold_mm']} | {'유지' if r['held'] else '실패'} |")
    open(os.path.join(out,"gripper_slip.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

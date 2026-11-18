# week09 박종진 — BC 평가용 「동역학 랜덤화」 (임시판)
#   ⚠ 9주차 도메인 랜덤화 · 민감도 분석의 원본은 함태훈 randomize.py(EventTerm 매핑 포함). 이 파일은 BC 평가를 먼저 돌리기 위한 같은 개념의 축소판이며,
#     함태훈 randomize.py가 push되면 apply()만 그쪽 함수로 바꿔 eval_bc.py를 다시 돌린다(seed → 같은 표본 규칙 유지)
#   범위 근거 = 가정값 표(b1/models/assumptions-v0.md) — 실측이 없는 값은 넓게, A2 사양이 있는 값은 좁게
#   seed가 같으면 같은 표본(전문가 · BC가 같은 랜덤 환경에서 비교됨)
import numpy as np,mujoco

RANGES={   # 이름: (범위, 근거)
 "friction_scale":   ((0.7,1.3),  "S16 마찰 1.0 · 0.8 = 가정(실물 마찰 미측정) → ±30 %, 손가락 · 물체 · 테이블 각각 따로"),
 "grip_force":       ((30.0,50.0),"S15 손가락당 40 N 가정(그리퍼 사양 미수령) → ±25 %"),
 "arm_armature":     ((0.8,1.2),  "S3 A2 회신 로터 관성만(기어열 TBD) → ±20 %"),
 "arm_damping":      ((0.5,2.0),  "S4 0.5 N·m·s/rad = 안정화용 작은 값, 근거 없음 → ×0.5 ~ 2"),
 "arm_kp":           ((0.9,1.1),  "S12 kp 고정 게인, 실기 서보 게인 미정 → ±10 %"),
 "base_mass":        ((0.85,1.15),"S10 하체 30 kg = 총중량 80 kg에서 역산(실측 보류) → ±15 %"),
 "base_kv":          ((0.8,1.2),  "S12 베이스 속도 게인 kv 200 · 50 = 함태훈 week05 응답 τ 0.38 s 기준 → ±20 %"),
}

def sample(seed):
    r=np.random.default_rng(10_000+int(seed)); u=lambda k:float(r.uniform(*RANGES[k][0]))
    return dict(fric_pad=u("friction_scale"),fric_obj=u("friction_scale"),fric_table=u("friction_scale"),grip_force=u("grip_force"),
                arm_armature=u("arm_armature"),arm_damping=u("arm_damping"),arm_kp=u("arm_kp"),base_mass=u("base_mass"),base_kv=u("base_kv"))

def apply(m,seed):
    """m(복사본)을 seed 표본으로 바꿈 → 표본 dict. 물체 질량은 원래 과제 무작위(0.2 ~ 3 kg)에 이미 포함"""
    p=sample(seed)
    for g in ("left_pad_l","left_pad_r"): m.geom_friction[m.geom(g).id,0]*=p["fric_pad"]
    for g in ("obj_box","obj_cyl"): m.geom_friction[m.geom(g).id,0]*=p["fric_obj"]
    m.geom_friction[m.geom("table").id,0]*=p["fric_table"]
    for n in ("left_finger_l","left_finger_r"): m.actuator_forcerange[m.actuator(n).id]=[-p["grip_force"],p["grip_force"]]
    for side in ("left","right"):
        for a in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"):
            dof=m.jnt_dofadr[m.joint(f"{side}_{a}_joint").id]; m.dof_armature[dof]*=p["arm_armature"]; m.dof_damping[dof]*=p["arm_damping"]
            i=m.actuator(f"{side}_{a}").id; m.actuator_gainprm[i,0]*=p["arm_kp"]; m.actuator_biasprm[i,1]*=p["arm_kp"]
    b=m.body("base_link").id; m.body_mass[b]*=p["base_mass"]; m.body_inertia[b]*=p["base_mass"]
    for n in ("base_vx","base_vy","base_wz"):
        i=m.actuator(n).id; m.actuator_gainprm[i,0]*=p["base_kv"]; m.actuator_biasprm[i,2]*=p["base_kv"]
    return p

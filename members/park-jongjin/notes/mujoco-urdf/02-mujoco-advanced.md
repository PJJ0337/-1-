# MuJoCo 심화 — MJCF 구성 · 구동 · URDF 로딩 (week03 공통 학습)

> 원 자료: 김이겸, Notion 「B1 2·3·4주차 학습 커리큘럼」 3주차(10/6 변경본: USD · Isaac Sim → MuJoCo 대응). 근거 문서는 [MuJoCo 문서](https://mujoco.readthedocs.io/en/stable/) (MuJoCo 3.15.0 기준), 실험은 [week03/scripts/](../../week03/scripts/).

## 1. USD · Isaac Sim 항목 → MuJoCo 대응 (커리큘럼 표 + 실습 결과)

| 원래 주제 | MuJoCo 대응 | week03 실습 |
|---|---|---|
| Stage · Prim · Attribute | `compiler`(angle, meshdir) · `option`(timestep, integrator, gravity) · `asset` · `worldbody` | [scene.xml](../../week03/mjcf/scene.xml) |
| 합성(sublayer · reference) | `<include>`, `<default class>`, `mjSpec` attach | 팔/scene 분리, 양팔 attach ([holonomic_base_two_arms.xml](../../week03/mjcf/holonomic_base_two_arms.xml)) |
| RigidBody · Joint · ArticulationRoot | body · joint · freejoint, 고정 vs 자유 베이스 | thu E5 |
| DriveAPI stiffness · damping | `position` kp · kv, `velocity`, gear, ctrlrange · forcerange, armature · damping | wed 계단 응답 |
| Importer 옵션 | URDF `<mujoco><compiler fusestatic · discardvisual · balanceinertia/>`, exclude | thu E2 · E3 |

## 2. 화 — MJCF 구성, URDF → MJCF

- MuJoCo 권장 절차: URDF에 확장 추가 → 로드 → MJCF 저장 → 추가 정보는 include ([Modeling › URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf))
- URDF와 MJCF는 4개 compiler 기본값이 다름: `angle`(URDF 항상 radian / MJCF degree), `fusestatic`(URDF true), `discardvisual`(URDF true), `strippath`
- URDF 변환 결과: effort → 관절 `actuatorfrcrange`, velocity 한계는 버려짐, inertial rpy → `quat` + `diaginertia`
- 손 작성 MJCF와 변환본 비교: 질량 · COM · 관성(오차 ≤ 1e-13) · 관절 축 · 범위 · damping 일치, 수평 중력토크 3.9731 / 0.7358 Nm, 낙하 궤적 차 1.2e-14 rad
- Euler와 implicitfast 결과가 같았던 이유: 이 모델의 속도 의존 힘은 관절 damping 뿐인데, "our Euler integrator handles damping implicitly" ([body-joint-damping](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-damping))

## 3. 수 — 구동

- position 액추에이터 힘 `f = kp·(ctrl − q) − kv·q̇` (gainprm[0] = kp, biasprm = [0, −kp, −kv]) → 관절 기준 스프링-댐퍼 = Isaac Drive stiffness · damping과 같은 구조
- 1자유도 근사: `ωn = √(kp / M)`, `ζ = (kv + damping) / (2√(kp·M))`, M = 질량행렬 대각 + armature
- armature = J_rotor × N² (감속기 반사 관성) → M 증가 → ωn 감소, 같은 kv에서 ζ 감소
- 정상상태 오차 ≈ τ_gravity / kp (중력보상 없는 P 서보)

| 실험 | ζ | 오버슈트 시뮬 / 이론 | 정착 2 % | 정상상태 오차 (이론) |
|---|---|---|---|---|
| kp 20 · kv 1 | 0.31 | 34.7 / 35.7 % | 1.00 s | 0.190 (0.174) rad |
| kp 50 · kv 1 | 0.20 | 51.2 / 53.2 % | 1.11 s | 0.072 (0.070) rad |
| kp 100 · kv 1 | 0.14 | 60.7 / 64.3 % (40 Nm 포화) | 1.03 s | 0.036 (0.035) rad |
| kp 50 · kv 5 | 0.95 | 0.0 / 0.0 % | 0.28 s | 0.072 rad |
| kp 50 · armature 0.15 | 0.14 | 61.9 / 64.7 % | 2.09 s | 0.070 rad |

## 4. 목 — URDF 로딩 옵션

- 체크리스트: [b1/docs/procedures/mujoco-urdf-loading-checklist.md](../../../../b1/docs/procedures/mujoco-urdf-loading-checklist.md)
- 자기 충돌: 부모-자식 접촉은 제외하지만 **부모가 world(용접 포함)면 제외 안 함** ([coSelection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection)) → 2주차 "어깨가 안 떨어짐" 원인 확정, `exclude`로 해결
- URDF joint type: revolute → hinge(limited), continuous → hinge(unlimited), prismatic → slide, **planar → slide · slide · hinge 3개**, floating → free

## 5. 구술 답변 (week03)

### Q1. 베이스를 고정할 때와 freejoint로 둘 때의 차이, 홀로노믹 베이스는 MJCF에서 어떻게 표현하는가?

- **고정**: 루트 body에 joint가 없으면 world에 용접된다(URDF 루트 링크 기본). 베이스는 움직이지 않고 팔 반력이 베이스 운동으로 나타나지 않는다. 자유도 = 팔 관절 수(양팔 4관절 모델: nq = nv = 4). 주의: world에 용접된 베이스와 첫 링크는 부모-자식 충돌 필터가 적용되지 않아 exclude가 필요하다.
- **freejoint**: 6자유도 추가(qpos 7 = 위치 3 + 쿼터니언 4, qvel 6 → nq 11 / nv 10). 중력으로 떨어지므로 바닥 · 바퀴 접촉과 마찰이 있어야 서 있고, 대신 팔을 뻗을 때의 기울어짐 · 전복 · 반력을 볼 수 있다. URDF의 floating joint도 free로 변환된다.
- **홀로노믹**: 베이스 body에 `slide x` · `slide y` · `hinge z` 3개를 두고 각각 velocity 액추에이터로 (vx, vy, ωz)를 명령한다(nq = nv = 7). 높이 · 롤 · 피치가 고정이라 넘어지지 않으며, 옴니휠 기구 없이 "평면 어디로든 이동"을 표현한다. URDF에서는 `planar` joint 1개로 쓰면 MuJoCo가 `_TX · _TY · _RZ`로 바꿔 준다.
- **핵심 수치 · 주의**: slide 축이 yaw보다 먼저 적용돼 **world 고정 축**이다 — 90° 회전 후 base_x 0.3 m/s 명령은 world x로 0.25 m 이동(로봇 기준 옆걸음). 로봇 기준 명령은 `R(yaw)`로 변환해서 넣어야 한다. 한계: 바퀴 슬립 · 가속 시 전복 모멘트는 표현 못 하므로 그 검증은 freejoint + 휠 모델로 따로 한다.

### Q2. `fusestatic`을 켜면 무엇이 사라지고, 언제 꺼야 하는가?

- **무엇**: joint 없이 부모에 붙은 정적 body가 부모와 합쳐지고, 그 안의 geom · site 등은 부모로 옮겨진다. 사라지는 것은 그 body의 **이름과 별도 좌표계**다(2링크: base_link · tool0, body 5 → 3). 질량 · 관성은 부모에 합산돼 동역학은 같다(중력토크 3.9731 Nm 동일). 다른 요소가 참조하는 body, force · torque 센서 site가 있는 body는 합치지 않는다 ([compiler-fusestatic](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-fusestatic)) — 그런데 URDF에는 참조 수단이 없으니 URDF에서는 fixed 링크가 전부 사라진다.
- **기본값**: URDF true, MJCF false.
- **꺼야 할 때**: ① 센서 · 툴 · 카메라 장착 프레임을 이름으로 써야 할 때(Isaac Lab FrameTransformer · 센서와 같은 이유), ② scene MJCF에서 `exclude` · `attach` · site 추가 대상으로 그 body가 필요할 때 — 고정 베이스의 base_link ↔ link1 exclude가 그 예, ③ 변환 결과를 URDF와 링크 단위로 비교 검증할 때. 켜 둬도 되는 때: 검증이 끝난 학습용 모델에서 body 수를 줄여 속도를 얻고 싶을 때(이 경우 필요한 프레임은 site로 미리 옮겨 둔다).

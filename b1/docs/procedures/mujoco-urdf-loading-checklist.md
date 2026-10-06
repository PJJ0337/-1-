# 우리 로봇 URDF → MuJoCo 로딩 체크리스트 (초안 v0)

| 항목 | 내용 |
|---|---|
| 상태 | **초안 v0.1** (week03, 박종진, 2026-10-06) — [함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §4 · §5 반영(G절). 함태훈 notes/07 §5 제안 8개 반영 대기 |
| 대상 | MuJoCo 3.15.0 (`pip install mujoco`), 우리 로봇 = 6-DOF 팔 2개 + 홀로노믹 베이스 |
| 배경 | 커리큘럼 10/6 변경: 실습 PC 전까지 Isaac Sim 임포트 대신 MuJoCo 로딩으로 같은 개념을 익힘. Isaac Sim용은 [urdf-import-checklist.md](urdf-import-checklist.md) (실습 PC 배정 후) |
| 문서 근거 | [MuJoCo Modeling — URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf) · [XML Reference — compiler](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler) · [Computation — Collision selection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection) · [XML Reference — actuator/position](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position) |
| 실험 근거 | [tue_urdf_to_mjcf.py](../../../members/park-jongjin/week03/scripts/tue_urdf_to_mjcf.py) · [wed_step_response.py](../../../members/park-jongjin/week03/scripts/wed_step_response.py) · [thu_loading_experiments.py](../../../members/park-jongjin/week03/scripts/thu_loading_experiments.py) → 결과 [results/](../../../members/park-jongjin/week03/results/) |

## 0. 권장 절차 (MuJoCo 문서 권장 방식)

> "Introduce extensions in the URDF as needed, load it and save it as MJCF. Then add information to the MJCF using include elements whenever possible." — [Modeling > URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf)

1. URDF에 `<mujoco><compiler .../></mujoco>` 확장 추가 (B절)
2. MuJoCo로 로드 → `mj_saveLastXML`로 `robot.xml`(MJCF) 저장 → **이 파일은 손대지 않음**
3. `scene.xml`에서 `<include file="robot.xml"/>` 후 액추에이터 · 센서 · 바닥 · exclude 를 추가 (E절)
4. F절 검증 → URDF가 바뀌면 1~2만 다시 실행

2링크 팔로 검증 완료: URDF 변환본과 손 작성 MJCF의 질량 · 관성 · 관절 축 · 범위 · damping 일치, 수평 중력토크 3.9731 / 0.7358 Nm, 2 s 낙하 궤적 차이 1.2e-14 rad ([tue_compare.txt](../../../members/park-jongjin/week03/results/tue_compare.txt))

### 0-1. URDF에 넣을 것 / MJCF에서만 넣을 것 ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §5 결론 1 · 2)

| URDF (단일 원본, A팀 CAD · BOM에서 옴) | MJCF에서만 (변환본에 include로 덧붙임) |
|---|---|
| 메시 · 질량 · COM · 관성, joint origin · axis · range · effort, `<mujoco><compiler>` 확장 | `armature`, `frictionloss`(URDF friction도 가능), `<actuator>`(kp · kv · gear · ctrlrange · forcerange), `<sensor>`, `<option>`, `<default>`, 충돌 필터(exclude · contype), 바닥 · 조명 |

## A. 로드 전 — URDF 자체 점검

| No. | 점검 | 기준 | 안 하면 (근거) |
|---|---|---|---|
| A1 | 움직이는 모든 link에 `<inertial>` | mass · origin · inertia 6성분 | `inertiafromgeom="auto"`: inertial 없으면 geom에서 추정(밀도 1000) → 0.2×0.1×0.1 상자가 2.0 kg (week02 실험). [compiler-inertiafromgeom](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom) |
| A2 | 가상 링크(센서 프레임 등)도 질량이 0이면 **fixed joint로만** 연결 | 움직이는 가상 링크엔 mass ≥ 1e-3 | 움직이는 body 질량 0 → `mass and inertia of moving bodies must be larger than mjMINVAL` 컴파일 오류 (week02 실험) |
| A3 | 관성 삼각부등식 A+B ≥ C | CAD 추출값 검사 | 위반 시 컴파일 오류. 급하면 `balanceinertia="true"`(평균값으로 바꿈 — 값이 바뀌므로 기록 필수). 실험: (0.02, 0.0075, 0.0002) → 오류 → balanceinertia 후 0.00923 × 3 (함태훈 · 박종진 재확인 일치). [compiler-balanceinertia](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-balanceinertia) |
| A4 | 단위 m · kg · rad, mm STL은 `scale="0.001 0.001 0.001"` | | 1000배 로봇 |
| A5 | 모든 revolute에 `<limit lower upper effort velocity>` | effort = 구동기 토크 | **effort → `actuatorfrcrange`(관절 단위 힘 제한)로 변환, velocity는 버려짐** (tue 실험: `two_link_from_urdf.xml`). 속도 제한은 MJCF 액추에이터/제어기 쪽에서 별도 처리 |
| A6 | 폐루프 없음 (트리) | | URDF · MJCF body 트리는 루프 표현 불가 → equality 제약으로 별도 |
| A8 | `<transmission>` · `<gazebo>`(ROS 전용)에 기대지 않음 | 액추에이터는 E절에서 MJCF로 추가 | MuJoCo가 무시 → 변환 후 `nu = 0`, 뷰어 Control 패널 없음 ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §4, 본인 재확인: 태훈 URDF 로드 nu = 0) |
| A9 | `<dynamics damping friction>` 값 확인 | damping → joint `damping`, friction → joint `frictionloss` | friction 0이면 변환본에서 생략됨 ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §3) |
| A7 | **MuJoCo는 URDF를 스키마 검사하지 않음** — `<mujoco>` 확장 안 속성 오타 확인 | 로드 후 `mj_saveLastXML` 결과에 반영됐는지 확인 | "mis-typed attribute names are silently ignored" ([Modeling > URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf)) |

## B. `<mujoco><compiler>` 확장 — URDF 기본값이 MJCF와 다른 4개 + α

> "the compiler attributes strippath, angle, fusestatic and discardvisual have different default values for URDF and MJCF" ([Modeling > URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf))

| No. | 속성 | URDF 기본 | 우리 로봇 권장 | 근거 · 실험 |
|---|---|---|---|---|
| B1 | **fusestatic** | **true** | **false** (센서 · 툴 · 장착 프레임 보존) | 정적 body(joint 없는 자식)를 부모에 합침. 동역학은 동일(중력토크 3.9731 Nm 그대로)하지만 **base_link · tool0 body가 사라짐** (body 5 → 3, thu E3) → 이후 MJCF에서 site · sensor · exclude · attach 대상으로 이름 참조 불가. 합쳐진 링크의 질량(함태훈 URDF: 고정 베이스 5 kg)도 world로 흡수 — 베이스가 움직이는 모델(C2 · C3)에선 joint가 있으니 합쳐지지 않지만, 베이스 위 고정 부품 질량은 베이스 body로 합쳐져 이름이 사라짐. [compiler-fusestatic](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-fusestatic) |
| B2 | **discardvisual** | **true** | 검증 단계 false / 학습 단계 true | `contype=conaffinity=0` geom · 재질 · 텍스처 삭제 → 2링크 visual 3개 소실, 베이스가 화면에서 사라짐 (tue 캡처 a). 동역학은 동일. [compiler-discardvisual](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-discardvisual) |
| B3 | **angle** | **항상 radian** | MJCF 쪽 파일엔 `angle="radian"` **명시** | MJCF 기본은 **degree** — 빼먹으면 range 1.5708이 ±1.5708°(0.0274 rad)로 해석 (thu E4). [compiler-angle](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-angle) |
| B4 | strippath | true(URDF) | 그대로 + **meshdir** 지정 | `package://` 경로 정보 제거 → 파일명만 남으므로 `meshdir`로 메시 폴더 지정 필수. [compiler-meshdir](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-meshdir) |
| B5 | inertiafromgeom | auto | auto (A1 지키면 영향 없음) | 공개 URDF 관성이 이상할 때만 true. [compiler-inertiafromgeom](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom) |
| B6 | balanceinertia | false | false (A3 위반 시만) | B 표 위 A3 |
| B7 | autolimits | true | true | range 있으면 limited 자동 |

예시 (우리 로봇 URDF 맨 위):
```xml
<robot name="b1_humanoid">
  <mujoco>
    <compiler meshdir="meshes/" fusestatic="false" discardvisual="false"/>
  </mujoco>
  ...
```

## C. 베이스 표현 — 고정 / freejoint / 홀로노믹

| No. | 표현 | MJCF | nq / nv (팔 2개 4관절 기준) | 언제 | 근거 · 실험 |
|---|---|---|---|---|---|
| C1 | **고정** | 루트 body에 joint 없음 = world에 용접 (URDF 루트 링크 기본) | 4 / 4 | 팔 단독 검증 · 매니퓰레이션 학습 초기 | thu E5 |
| C2 | **freejoint** | `<freejoint/>` (URDF `floating` joint도 free로 변환됨) | 11 / 10 (위치 3 + 쿼터니언 4) | 바퀴-바닥 접촉 · 전복 · 가속 시 기울어짐까지 볼 때. 바퀴 모델 · 마찰 필요 | thu E1 · E5, [body-freejoint](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-freejoint) |
| C3 | **홀로노믹(평면 3-DOF)** | 베이스 body에 `slide x` · `slide y` · `hinge z` 3개 + **velocity 액추에이터** | 7 / 7 | **우리 로봇 1차 권장**: 옴니휠 기구를 빼고 "평면 어디로든 이동" 만 표현. 높이 · 롤 · 피치 고정 | thu E5 · E6 |
| C3-1 | URDF로 쓸 때 | `type="planar"` 1개 → MuJoCo가 `_TX`·`_TY`(slide) + `_RZ`(hinge) 로 자동 변환 | 3 / 3 | 더미 링크 2개 없이 URDF 1관절로 표현 가능 | thu E1 |
| C3-2 | **명령 좌표계** | slide 축은 yaw보다 **먼저** 적용 → world 고정 축 | | 90° 돈 뒤 `base_x` 0.3 m/s → world x로 이동(로봇 기준 옆걸음). body 기준 명령은 `[vx_w, vy_w] = R(yaw)·[vx_b, vy_b]` 변환 필요 | thu E5, [thu_holonomic.csv](../../../members/park-jongjin/week03/results/thu_holonomic.csv) |
| C3-3 | 베이스 치수 | 상판 575 × 472 × 12 mm (BASE 3.STEP, 노션 4주차 미리보기) + 하체 박스 임시값 | | 하체 실측표(함태훈 week02)로 질량 · 높이 교체 예정 | [holonomic_base_two_arms.xml](../../../members/park-jongjin/week03/mjcf/holonomic_base_two_arms.xml) |

## D. 자기 충돌

| No. | 점검 | 방법 | 근거 · 실험 |
|---|---|---|---|
| D1 | 부모-자식 body 접촉은 **기본 제외**, 단 **부모가 world(또는 world에 용접된 body)면 제외 안 됨** | 고정 베이스 로봇에서 base ↔ 첫 링크 충돌 확인 | Collision selection 필터 3: "cannot belong to a parent and a child body, unless the parent is the world body … bodies welded together … treated as a single body" ([coSelection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection)). 2주차 "어깨가 수평에서 안 떨어짐"의 원인 — fusestatic 끄든 켜든 접촉 발생 (3.14: 1개, 3.15: 2개 — 원통-원통 충돌 계산이 버전별로 다름) (thu E2) |
| D2 | 해결: scene MJCF에 `<contact><exclude body1="base_link" body2="link1"/></contact>` | body 이름이 남아 있어야 함 → **B1 fusestatic=false 필요** | exclude 후 접촉 0개, 어깨 +1.575 rad까지 정상 낙하 (thu E2). [contact-exclude](https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-exclude) |
| D3 | 팔-몸통, 왼팔-오른팔처럼 **부모-자식이 아닌** 쌍은 기본 충돌함 | 필요 없는 쌍만 exclude, 넓게 끄려면 `contype`/`conaffinity` 비트 | `(contype1 & conaffinity2) || (contype2 & conaffinity1)` (coSelection 필터 4) |
| D4 | visual geom은 `contype=conaffinity=0` | default class `visual`로 일괄 | [two_link_arm.xml](../../../members/park-jongjin/week03/mjcf/two_link_arm.xml) |

## E. 액추에이터 추가 (URDF에는 액추에이터가 없음)

| No. | 항목 | 권장 | 근거 · 실험 |
|---|---|---|---|
| E1 | 위치 | 변환 MJCF는 그대로 두고 **scene 쪽 `<actuator>`** 에 추가, `<default class>`로 kp · kv 공통화 | [scene_position.xml](../../../members/park-jongjin/week03/mjcf/scene_position.xml) |
| E2 | 팔 관절 | `position` (kp, kv) — 힘 `f = kp·(ctrl − q) − kv·q̇` | [actuator-position](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position) |
| E3 | 베이스 평면 관절 | `velocity` (kv) | thu E5 |
| E4 | `forcerange` = 구동기 토크 (URDF effort), `ctrlrange` = 관절 range — position은 `inheritrange="1"`로 관절 range 자동 상속. URDF effort는 이미 joint `actuatorfrcrange`로 들어와 있어 **관절 쪽 · 액추에이터 쪽 두 군데서 제한** (관절 1개당 액추에이터 1개면 같은 효과, [CForceRange](https://mujoco.readthedocs.io/en/stable/modeling.html#cforcerange)) | 포화 확인 | kp 100에서 40 Nm 포화 → 이론 오버슈트 64.3 % vs 시뮬 60.7 % (wed) |
| E5 | 감쇠 설정 | kv 대신 `dampratio`(1 = 임계감쇠) 고려, kv 쓰면 `implicitfast` 적분기 | 문서: kv 사용 시 implicitfast/implicit 권장. kv 5(ζ 0.95) → 오버슈트 0 %, 정착 0.28 s (wed) |
| E6 | **armature** = J_rotor × N² (감속기 반사 관성) | BOM 구동기 로터 관성 · 감속비로 계산해 관절에 입력 | armature 0 → 0.15: 유효관성 0.142 → 0.292 kg·m², ωn 18.8 → 13.1 rad/s (wed). [body-joint-armature](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature) |
| E7 | 중력 | position 서보는 중력보상 없으면 **정상상태 오차 ≈ τg / kp** | kp 50: 0.072 rad (이론 0.070) (wed) |

## E-1. MJCF를 손으로 쓸 때 (변환본을 고치지 않고 새로 쓸 경우)

| No. | 점검 | 근거 |
|---|---|---|
| H1 | cylinder · capsule `size` = **반지름, 반길이** (URDF `length`의 절반) — 또는 `fromto`로 양 끝점 지정 | [함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §2 (0.15 = 0.3/2), 박종진 two_link_arm.xml은 fromto 사용 |
| H2 | `compiler angle="radian"` 명시 (B3) | thu E4 |
| H3 | URDF 변환본과 질량 · 관성 · 중력토크 자동 비교 | tue_urdf_to_mjcf.py 방식 |

## F. 로드 후 검증

| No. | 점검 | 기준 |
|---|---|---|
| F1 | body · joint 이름 목록 = URDF (fusestatic=false 기준) | 누락 없음 |
| F2 | 총질량 = URDF 합계 (목표 ≤ 80 kg) | |
| F3 | 수평 자세 중력토크 = 손계산 | 2링크: 3.9731 / 0.7358 Nm |
| F4 | `mj_saveLastXML` 결과에서 확장 속성 반영 확인 (A7) | |
| F5 | 뷰어: 영점 자세 정지 · 관절 범위 양 끝 · 중력 낙하 | `python -m mujoco.viewer --mjcf=scene.xml` |

## G. 팀 연계 — 함태훈 URDF ↔ MJCF 대응표 반영

출처: [함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) (week03, 함태훈), 반영 2026-10-06

| 대응표 항목 | 체크리스트 반영 |
|---|---|
| §5 결론 1 · 2: URDF = 단일 원본, MJCF 전용 항목은 include로 | 0-1절 표 신설 |
| §3 `<transmission>` 무시 → nu = 0 | A8 신설 |
| §3 friction → frictionloss | A9 신설 |
| §2 관성 삼각부등식 · balanceinertia 실험값 | A3 보강 |
| §4 fusestatic로 베이스 질량(5 kg) 흡수 | B1 보강 |
| §3 effort → actuatorfrcrange, forcerange 별개 · inheritrange | E4 보강 |
| §2 실린더 size = 반길이 | E-1절 H1 신설 |
| §5 결론 3: `<mujoco><compiler>` 확장을 ICD 규약에 | 동의 — B절 예시 블록을 ICD v1 개정 때 제안 (4주차) |
| §5 결론 4: kp → Drive stiffness, kv → damping, armature → joint armature | 아래 "실습 PC 배정 후" 절 |
| §1 · §3 나머지(단위 · 좌표계 · velocity 한계 · 부모-자식 충돌 예외) | 기존 A5 · B3 · D1과 일치 확인 |

- [ ] 함태훈 notes/07 §5 체크리스트 제안 8개 반영
- [ ] 함태훈이 이 체크리스트로 `sim/urdf/two_link_arm.urdf` · `sim/mjcf/two_link_arm_v2.xml` 점검 → 결과 링크

## 실습 PC 배정 후 (Isaac Sim)

- 액추에이터 대응: MuJoCo position `kp` → Drive stiffness, `kv` → Drive damping, `armature` → joint armature ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §5-4) — 단위(rad vs deg)와 Drive Type(Force/Acceleration) 확인은 [urdf-import-checklist.md](urdf-import-checklist.md) B4
- 같은 URDF를 Isaac Sim 5.1 URDF Importer로 임포트 → [urdf-import-checklist.md](urdf-import-checklist.md) C절, 중력토크 · 말단 위치를 F3과 비교

# 우리 로봇 URDF → MuJoCo 로딩 체크리스트 (초안 v0)

| 항목 | 내용 |
|---|---|
| 상태 | **v0.4.1** (week05, 박종진, 2026-10-18) — **0-3절 빌드 규약 확정**(함태훈 동의 10/17), E7-1 중력보상 A · B 구분(함태훈 발견: 목표 오프셋은 ctrlrange에 잘림), gravcomp 옵션. **v0.4** (2026-10-16) — **v1 기준 갱신**: 0-3절 빌드 스크립트 규약(제안), E8 액추에이터 응답 · M절 좌우 미러 · F6 검증 5종 추가, 자동 점검 [checklist_v1.md](../../models/v1/results/checklist_v1.md) 28항목 OK. **v0.3.1** (2026-10-16) — C3 평면 3-DOF 채택 · C3-1 Isaac Sim 분해 비고(팀장 10/16). **v0.3** (week04, 2026-10-13) — **B6 balanceinertia 기본 off 확정**(김이겸 10/8 회신), A3 기록 규칙 추가. 이전: **v0.2** (week03, 2026-10-06) — 함태훈 notes/05 대응표 · [함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §5 제안 8개 반영, 함태훈 파일 점검 완료(G절), **팀 규약: 3단 파일 구조(0-2절)** |
| 대상 | MuJoCo 3.15.0 (`pip install mujoco`), 우리 로봇 = 4-DOF 팔 2개(손목 없음, 10/16 확정) + 홀로노믹 베이스(C3) |
| 배경 | 커리큘럼 10/6 변경: 실습 PC 전까지 Isaac Sim 임포트 대신 MuJoCo 로딩으로 같은 개념을 익힘. Isaac Sim용은 [urdf-import-checklist.md](urdf-import-checklist.md) (실습 PC 배정 후) |
| 문서 근거 | [MuJoCo Modeling — URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf) · [XML Reference — compiler](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler) · [Computation — Collision selection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection) · [XML Reference — actuator/position](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position) |
| 실험 근거 | [tue_urdf_to_mjcf.py](../../../members/park-jongjin/week03/scripts/tue_urdf_to_mjcf.py) · [wed_step_response.py](../../../members/park-jongjin/week03/scripts/wed_step_response.py) · [thu_loading_experiments.py](../../../members/park-jongjin/week03/scripts/thu_loading_experiments.py) → 결과 [results/](../../../members/park-jongjin/week03/results/) |

## 0. 권장 절차 (MuJoCo 문서 권장 방식)

> "Introduce extensions in the URDF as needed, load it and save it as MJCF. Then add information to the MJCF using include elements whenever possible." — [Modeling > URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf)

1. URDF에 `<mujoco><compiler .../></mujoco>` 확장 추가 (B절)
2. MuJoCo로 로드 → `mj_saveLastXML`로 `robot.xml`(MJCF) 저장 → **이 파일은 손대지 않음**
3. `robot_actuated.xml`에서 `<include file="robot.xml"/>` 후 액추에이터 · 센서 · exclude · armature 추가 → `scene.xml`에서 `<include file="robot_actuated.xml"/>` 후 바닥 · 조명 · option (0-2절)
4. F절 검증 → URDF가 바뀌면 1~2만 다시 실행

2링크 팔로 검증 완료: URDF 변환본과 손 작성 MJCF의 질량 · 관성 · 관절 축 · 범위 · damping 일치, 수평 중력토크 3.9731 / 0.7358 Nm, 2 s 낙하 궤적 차이 1.2e-14 rad ([tue_compare.txt](../../../members/park-jongjin/week03/results/tue_compare.txt))

### 0-1. URDF에 넣을 것 / MJCF에서만 넣을 것 ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §5 결론 1 · 2)

| URDF (단일 원본, A팀 CAD · BOM에서 옴) | MJCF에서만 (변환본에 include로 덧붙임) |
|---|---|
| 메시 · 질량 · COM · 관성, joint origin · axis · range · effort, `<mujoco><compiler>` 확장 | `armature`, `frictionloss`(URDF friction도 가능), `<actuator>`(kp · kv · gear · ctrlrange · forcerange), `<sensor>`, `<option>`, `<default>`, 충돌 필터(exclude · contype), 바닥 · 조명 |

### 0-2. 팀 규약 — 3단 파일 구조 (박종진 · 함태훈 합의, 2026-10-06, 4주차 우리 로봇 v0부터 적용)

| 단계 | 파일 | 내용 | 수정 |
|---|---|---|---|
| 1 | `robot.xml` | URDF → `mj_saveLastXML` 변환본 | **수정 금지** (URDF가 바뀌면 재생성) |
| 2 | `robot_actuated.xml` | `<include file="robot.xml"/>` + `<actuator>` · `<sensor>` · `<contact><exclude>` · joint 보강(armature · frictionloss, `<default>`) | 손으로 관리 |
| 3 | `scene.xml` | `<include file="robot_actuated.xml"/>` + 바닥 · 조명 · 카메라 · `<option>` | 손으로 관리 |

- 근거: MuJoCo 권장(변환본은 그대로, 추가 정보는 include — 0절) + menagerie 구조(로봇 파일 = 액추에이터 포함, scene = 환경만). 2단계 파일만 열어도 구동되는 로봇, 3단계는 실험 환경.
- 제안: 함태훈 ([함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) 피드백). week03 2링크 파일은 각자 기존 구조 유지.

### 0-3. v1 규약 — 빌드 스크립트로 3단 구조 자동화 (week05 커리큘럼 「팀 연계」) — **확정 (박종진 제안 10/16 · 함태훈 동의 10/17)**

| 단계 | 파일 (b1/models/v1/) | 내용 | 수정 |
|---|---|---|---|
| 0 | `urdf/b1_v1.urdf` · `urdf/b1_v1_isaac.urdf` | `scripts/build_v1.py`가 `assumptions_b1.json`에서 생성 (원본 = planar, Isaac 사본 = prismatic · prismatic · revolute) | **생성물 — 손으로 수정 금지**, 값은 json에서 |
| 1 | `mjcf/b1_urdf.xml` | URDF → `mj_saveLastXML` 변환본 (0-2절 1단계와 같음) | 수정 금지 |
| 2 | `mjcf/b1.xml` | 변환본 + site(ee · IMU) + `default class`(shoulder · elbow · base) + 액추에이터 + 센서 — **`actuator_params.json`에서 생성** | params만 수정 (**함태훈 담당**) |
| 3 | `mjcf/scene.xml` | `<include file="b1.xml"/>` + 바닥 · 조명 · option | 생성 |

- 0-2절과 차이: 2단계를 손으로 관리하던 것을 **params 파일 + 빌드**로 바꿈 → URDF를 다시 만들어도 덧붙이기가 사라지지 않음. site는 body 안에 있어야 해서 include로 덧붙일 수 없음 → 빌드가 변환본을 읽어 b1.xml에 넣음 (변환본 자체는 그대로)
- 입력: 링크 질량 · 관성 = `inputs/link_inertials_v1.json`(함태훈, 원점 = 골격 관절점 → 평행이동 변환 없음), 덧붙이기 = `actuator_params.json`(함태훈 관리, 근거 포함). 중력보상 B는 `actuator_params.json` `gravcomp.enable`(켜면 팔 body gravcomp · joint actuatorgravcomp + `jointactuatorfrc` 센서 8개)
- 순서: `python b1/models/v1/scripts/build_v1.py` → `validate_model.py` → `render_v1.py` → `checklist_v1_check.py`

## A. 로드 전 — URDF 자체 점검

| No. | 점검 | 기준 | 안 하면 (근거) |
|---|---|---|---|
| A1 | 움직이는 모든 link에 `<inertial>` | mass · origin · inertia 6성분 | `inertiafromgeom="auto"`: inertial 없으면 geom에서 추정(밀도 1000) → 0.2×0.1×0.1 상자가 2.0 kg (week02 실험). [compiler-inertiafromgeom](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom) |
| A2 | 가상 링크(센서 프레임 등)도 질량이 0이면 **fixed joint로만** 연결 | 움직이는 가상 링크엔 mass ≥ 1e-3 | 움직이는 body 질량 0 → `mass and inertia of moving bodies must be larger than mjMINVAL` 컴파일 오류 (week02 실험) |
| A3 | 관성 삼각부등식 A+B ≥ C | CAD 추출값 검사 | 위반 시 컴파일 오류. **먼저 CAD · 관성 추출 좌표계(COM 기준인지, 링크 축 방향인지, mm²·g 단위인지)를 의심**하고 다시 뽑는다. 원인을 못 찾아 임시로 `balanceinertia="true"`를 켤 때는 **링크명 · 원래 값 · 바뀐 값 · 사유**를 그 주 log와 B절 아래 「A3 · B6 기록」에 남긴다 (B6, 팀 결정 10/8). 실험: (0.02, 0.0075, 0.0002) → 오류 → balanceinertia 후 0.00923 × 3 (함태훈 · 박종진 재확인 일치). [compiler-balanceinertia](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-balanceinertia) |
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
| B1 | **fusestatic** | **true** | **false** (센서 · 툴 · 장착 프레임 보존) | 정적 body(joint 없는 자식)를 부모에 합침. 동역학은 동일(중력토크 3.9731 Nm 그대로)하지만 **base_link · tool0 body가 사라짐** (body 5 → 3, thu E3) → 이후 MJCF에서 site · sensor · exclude · attach 대상으로 이름 참조 불가. 합쳐진 링크의 질량(함태훈 URDF: 고정 베이스 5 kg)도 world로 흡수 — 베이스가 움직이는 모델(C2 · C3)에선 joint가 있으니 합쳐지지 않지만, 베이스 위 고정 부품 질량은 베이스 body로 합쳐져 이름이 사라짐. **켜 둬도 되는 경우**: 고정 브래킷 · 커버 링크가 많은 CAD 기반 URDF에서 속도를 원할 때([함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §4) — 단 우리 로봇은 4주차 링크 분할에서 브래킷을 링크 메시에 미리 합치므로, 남는 고정 링크는 주로 센서 · 툴 프레임 → **false 권장 유지** [compiler-fusestatic](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-fusestatic) |
| B2 | **discardvisual** | **true** | 검증 단계 false / 학습 단계 true | `contype=conaffinity=0` geom · 재질 · 텍스처 삭제 → 2링크 visual 3개 소실, 베이스가 화면에서 사라짐 (tue 캡처 a). 동역학은 동일. [compiler-discardvisual](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-discardvisual) |
| B3 | **angle** | **항상 radian** | MJCF 쪽 파일엔 `angle="radian"` **명시** | MJCF 기본은 **degree** — 빼먹으면 range 1.5708이 ±1.5708°(0.0274 rad)로 해석 (thu E4). [compiler-angle](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-angle) |
| B4 | strippath | true(URDF) | 그대로 + **meshdir** 지정 | `package://` 경로 정보 제거 → 파일명만 남으므로 `meshdir`로 메시 폴더 지정 필수. [compiler-meshdir](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-meshdir) |
| B5 | inertiafromgeom | auto | auto (A1 지키면 영향 없음) | 공개 URDF 관성이 이상할 때만 true. [compiler-inertiafromgeom](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-inertiafromgeom) |
| B6 | balanceinertia | false | **false (확정)** | **팀 결정 2026-10-08 (김이겸)**: 기본 off. 켜면 CAD 관성값이 평균값으로 조용히 바뀌어 입력 오류를 못 잡는다. 삼각부등식 오류 시 CAD · 관성 추출 좌표계를 먼저 의심(A3), 임시로 켤 때는 바뀐 값과 사유를 A3 · B6에 기록. (논의 경과: 함태훈 제안 기본 true — [notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §5-1, 박종진 의견 false → false 채택) |
| B7 | autolimits | true | true | range 있으면 limited 자동 |

**A3 · B6 기록** (balanceinertia를 임시로 켠 경우만)

| 링크 | 원래 Ixx · Iyy · Izz | 바뀐 값 | 사유 | 날짜 · 기록자 |
|---|---|---|---|---|
| (없음) | | | | |

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
| C3 | **홀로노믹(평면 3-DOF)** — **B1 채택(팀장 10/16)** | 베이스 body에 `slide x` · `slide y` · `hinge z` 3개 + **velocity 액추에이터** | 7 / 7 | **우리 로봇 1차 권장**: 옴니휠 기구를 빼고 "평면 어디로든 이동" 만 표현. 높이 · 롤 · 피치 고정 | thu E5 · E6 |
| C3-0 | 대안: free + 힘 인가 | `<freejoint/>` + 베이스에 `<motor>`로 x · y · yaw 힘/토크 직접 인가 (옴니휠 추상화) | 11 / 10 | 기울어짐 · 바닥 접촉까지 보면서 휠은 생략할 때. 바퀴 4개 hinge + 접촉은 하체 실측 후 | [함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §3 |
| C3-1 | URDF로 쓸 때 | `type="planar"` 1개 → MuJoCo가 `_TX`·`_TY`(slide) + `_RZ`(hinge) 로 자동 변환 | 3 / 3 | 더미 링크 2개 없이 URDF 1관절로 표현 가능. **비고(팀장 10/16, C3 채택): URDF는 planar 1개 유지. Isaac Sim 이식 시 planar를 지원하지 않으므로 더미 링크 2개 + `prismatic x` · `prismatic y` · `revolute z` 3관절로 분해한 Isaac용 사본을 만든다(원본 URDF는 그대로)** | thu E1 |
| C3-2 | **명령 좌표계** | slide 축은 yaw보다 **먼저** 적용 → world 고정 축 | | 90° 돈 뒤 `base_x` 0.3 m/s → world x로 이동(로봇 기준 옆걸음). body 기준 명령은 `[vx_w, vy_w] = R(yaw)·[vx_b, vy_b]` 변환 필요 | thu E5, [thu_holonomic.csv](../../../members/park-jongjin/week03/results/thu_holonomic.csv) |
| C3-4 | 결정 순서 | 베이스 표현(C1 · C2 · C3)을 먼저 정하고 **fusestatic(B1)을 같이 결정** — 베이스를 나중에 free로 바꿀 계획이면 베이스 질량이 world로 흡수되지 않게 | | | [함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §5-2 |
| C3-3 | 베이스 치수 | 상판 575 × 472 × 12 mm (BASE 3.STEP, 노션 4주차 미리보기) + 하체 박스 임시값 | | 하체 실측표(함태훈 week02)로 질량 · 높이 교체 예정 | [holonomic_base_two_arms.xml](../../../members/park-jongjin/week03/mjcf/holonomic_base_two_arms.xml) |

## D. 자기 충돌

| No. | 점검 | 방법 | 근거 · 실험 |
|---|---|---|---|
| D1 | 부모-자식 body 접촉은 **기본 제외**, 단 **부모가 world(또는 world에 용접된 body)면 제외 안 됨** | 고정 베이스 로봇에서 base ↔ 첫 링크 충돌 확인 | Collision selection 필터 3: "cannot belong to a parent and a child body, unless the parent is the world body … bodies welded together … treated as a single body" ([coSelection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection)). 2주차 "어깨가 수평에서 안 떨어짐"의 원인 — fusestatic 끄든 켜든 접촉 발생 (3.14: 1개, 3.15: 2개 — 원통-원통 충돌 계산이 버전별로 다름) (thu E2) |
| D2 | 해결: `robot_actuated.xml`에 `<contact><exclude body1="base_link" body2="link1"/></contact>` — 또는 베이스 collision 자체를 빼거나 `contype=0`(함태훈 · 박종진 2주차 방식, 베이스 바닥 충돌이 필요 없을 때) | body 이름이 남아 있어야 함 → **B1 fusestatic=false 필요** | exclude 후 접촉 0개, 어깨 +1.575 rad까지 정상 낙하 (thu E2). [contact-exclude](https://mujoco.readthedocs.io/en/stable/XMLreference.html#contact-exclude) |
| D3 | 팔-몸통, 왼팔-오른팔처럼 **부모-자식이 아닌** 쌍은 기본 충돌함 | 필요 없는 쌍만 exclude, 넓게 끄려면 `contype`/`conaffinity` 비트 | `(contype1 & conaffinity2) || (contype2 & conaffinity1)` (coSelection 필터 4) |
| D4 | visual geom은 `contype=conaffinity=0` | default class `visual`로 일괄 | [two_link_arm.xml](../../../members/park-jongjin/week03/mjcf/two_link_arm.xml) |

## E. 액추에이터 추가 (URDF에는 액추에이터가 없음)

| No. | 항목 | 권장 | 근거 · 실험 |
|---|---|---|---|
| E1 | 위치 | **0-2절 팀 규약**: 변환본(robot.xml)은 그대로, `robot_actuated.xml`에 `<actuator>` · `<sensor>` 추가, `<default class>`로 kp · kv 공통화. scene은 환경만 | week03 2링크는 구 구조(scene_position.xml이 scene을 include) — 4주차부터 규약 적용 |
| E2 | 팔 관절 | `position` (kp, kv) — 힘 `f = kp·(ctrl − q) − kv·q̇` | [actuator-position](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position) |
| E3 | 베이스 평면 관절 | `velocity` (kv) | thu E5 |
| E4 | `forcerange` = 구동기 토크 (URDF effort), `ctrlrange` = 관절 range — position은 `inheritrange="1"`로 관절 range 자동 상속. URDF effort는 이미 joint `actuatorfrcrange`로 들어와 있어 **관절 쪽 · 액추에이터 쪽 두 군데서 제한** (관절 1개당 액추에이터 1개면 같은 효과, [CForceRange](https://mujoco.readthedocs.io/en/stable/modeling.html#cforcerange)) | 포화 확인 | kp 100에서 40 Nm 포화 → 이론 오버슈트 64.3 % vs 시뮬 60.7 % (wed) |
| E5 | 감쇠 설정 | kv 대신 `dampratio`(1 = 임계감쇠) 고려, kv 쓰면 `implicitfast` 적분기 | 문서: kv 사용 시 implicitfast/implicit 권장. kv 5(ζ 0.95) → 오버슈트 0 %, 정착 0.28 s (wed) |
| E6 | **armature** = J_rotor × N² (감속기 반사 관성) | **BOM 수령 전: 0 허용**(log에 "BOM 대기" 명시) / **BOM 수령 후: J_rotor × N² 필수** (함태훈 제안) | armature 0 → 0.15: 유효관성 0.142 → 0.292 kg·m², ωn 18.8 → 13.1 rad/s (wed). [body-joint-armature](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature) |
| E7 | 중력 | position 서보는 중력보상 없으면 **정상상태 오차 ≈ τg / kp** | kp 50: 0.072 rad (이론 0.070) (wed) |
| E7-1 | 중력보상 | **A 목표 오프셋**: position 목표에 τg / kp(τg = `qfrc_bias`, qvel ≈ 0) — 간단하지만 **목표가 ctrlrange(= 관절 범위, inheritrange)에 잘려** 한계 근처에서 처짐이 남음. **B `gravcomp` + `actuatorgravcomp`**: 팔 body `gravcomp="1"` + joint `actuatorgravcomp="true"` → 보상 토크가 actuator 쪽(qfrc_actuator)으로, actuatorfrcrange로만 제한 → **B 권장**. B의 보상분은 `actuatorfrc`에 안 잡히므로 토크 기록은 `jointactuatorfrc`. `body_gravcomp`는 컴파일 때 정해져 런타임 변경 불가 | 팔 수평 4.83° → A · B 0.000°, **roll 상한 1.5 rad: A 4.33°(클리핑) → B 0.000°** (함태훈 10/17 발견, [validate_model.md](../../models/v1/results/validate_model.md) ②) |
| E8 | 액추에이터 응답 | 액추에이터마다 범위 40 % 명령 → 관절이 같은 부호 · 크기로 응답(뷰어 Control 슬라이더 점검의 자동판) | v1 11/11 ([actuator_check.json](../../models/v1/results/actuator_check.json)) |

## E-1. MJCF를 손으로 쓸 때 (변환본을 고치지 않고 새로 쓸 경우)

| No. | 점검 | 근거 |
|---|---|---|
| H1 | cylinder · capsule `size` = **반지름, 반길이** (URDF `length`의 절반) — 또는 `fromto`로 양 끝점 지정 | [함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §2 (0.15 = 0.3/2), 박종진 two_link_arm.xml은 fromto 사용 |
| H2 | `compiler angle="radian"` 명시 (B3) | thu E4 |
| H3 | URDF 변환본과 질량 · 관성 · 중력토크 자동 비교 | 공용: 함태훈 [`checklist_check.py`](https://github.com/taehoonham0118-cell/-1-/blob/c441a5e/sim/scripts/checklist_check.py) (H3 부분), 박종진 tue_urdf_to_mjcf.py |

## F. 로드 후 검증

| No. | 점검 | 기준 |
|---|---|---|
| F1 | `mj_saveLastXML` 저장본의 body · geom · joint 수와 이름 = URDF (fusestatic=false 기준) | 누락 없음 ([함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §5-8) |
| F2 | 총질량 = URDF 합계 (목표 ≤ 80 kg) | |
| F3 | 수평 자세 중력토크 = 손계산 | 2링크: 3.9731 / 0.7358 Nm |
| F4 | `mj_saveLastXML` 결과에서 확장 속성 반영 확인 (A7) | |
| F5 | 뷰어: 영점 자세 정지 · 관절 범위 양 끝 · 중력 낙하 | `python -m mujoco.viewer --mjcf=scene.xml` |
| F6 | **모델 검증 5종** (week05): ① 질량 ±5 % ② 정지 유지 5 s ≤ 0.5° ③ 중력 토크 vs 정격 ④ 범위 sweep 자기 충돌 0 ⑤ 좌우 대칭 x · z ≤ 1 mm | [validate_model.md](../../models/v1/results/validate_model.md) — ③은 정격 미수령 → 판정 기준 정격 ≥ 52 N·m(그리퍼 box 가정 포함, 팀장 10/19) |

## M. 좌우 미러 (v1 양팔)

| No. | 점검 | 방법 | 근거 |
|---|---|---|---|
| M1 | 한쪽 팔만 정의, 다른 쪽은 **스크립트로 생성** | 장착 위치 · COM · 메시 y 반전, 축 = 유사벡터 변환(x · z 반전, y 유지), rpy roll · yaw 반전, 관성 ixy · iyz 반전 | [mirror_rules.md](../../models/v1/docs/mirror_rules.md) |
| M2 | 메시 거울은 **면 방향(winding)도 뒤집기** | y 반전만 하면 법선이 안쪽 → 렌더 · 체적 부호 오류 | build_v1.py `faces[:, ::-1]` |
| M3 | 같은 관절각에서 ee y 부호만 다름 (검증 ⑤) | 무작위 500자세 + 대표 4자세 | 차 0 mm (생성식이 정확한 거울) |

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

### G-2. [함태훈 notes/07](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/07_mjcf_structure_urdf_loading.md) §5 제안 8개 반영

| 제안 | 반영 |
|---|---|
| 1. `<mujoco><compiler meshdir discardvisual="false" balanceinertia="true">` | B절 예시 블록에 이미 있음. balanceinertia 기본값은 의견 차이 → B6 → **10/8 팀장 결정: 기본 off** (v0.3) |
| 2. 루트 고정/자유 결정 + fusestatic 함께 | C3-0(freejoint + motor 대안) · C3-4 신설, B1 보강 |
| 3. 모든 링크 inertial + 삼각부등식 | A1 · A3 (기존) |
| 4. 실린더 반길이, 메시 scale 0.001 | E-1 H1 · A4 (기존) |
| 5. effort → actuatorfrcrange, forcerange 별도, velocity 소실 | A5 · E4 (기존) |
| 6. transmission 무시 → actuator · armature · frictionloss는 MJCF에서 | A8 · 0-1 · 0-2 (기존 + 규약) |
| 7. 월드 고정 베이스-첫 링크 충돌 | D1 · D2 (contype 0 대안 추가) |
| 8. mj_saveLastXML 저장본으로 개수 대조 | F1 보강 |

### G-3. 함태훈 파일 점검 결과 (체크리스트 v0.1 기준, 함태훈 week03 태그 c441a5e)

- 대상: [`sim/urdf/two_link_arm.urdf`](https://github.com/taehoonham0118-cell/-1-/blob/c441a5e/sim/urdf/two_link_arm.urdf), [`sim/mjcf/two_link_arm_v2.xml`](https://github.com/taehoonham0118-cell/-1-/blob/c441a5e/sim/mjcf/two_link_arm_v2.xml) + `scene.xml`
- 결과: 36항목 — **OK 30 · NG 1 · N-A 5** → [notes/07 §6 번호별 표](https://github.com/taehoonham0118-cell/-1-/blob/c441a5e/notes/07_mjcf_structure_urdf_loading.md#6-박종진-체크리스트-v01-로-내-urdf--mjcf-점검-결과-107-팀-연계)
- 점검 스크립트: [`sim/scripts/checklist_check.py`](https://github.com/taehoonham0118-cell/-1-/blob/c441a5e/sim/scripts/checklist_check.py) — 박종진이 c441a5e 파일로 재실행(MuJoCo 3.15.0, 2026-10-06): 합계 동일 OK 30 · NG 1 · N-A 5, H3 중력토크 변환본 = v2 = 손계산 [5.886, 1.4715] N·m
- NG 1건 = **E6 armature 0** (BOM 전) → E6 조건("BOM 수령 전 0 허용 / 수령 후 J_rotor·N² 필수")으로 해소
- 점검하면서 함태훈이 고친 것: ① URDF에 `<mujoco><compiler fusestatic="false" discardvisual="false"/>` 추가 → 변환본 body 3 → 4, geom 2 → 5 (B1 · B2 · A7) ② v2 joint에 `actuatorfrcrange ±25` 추가해 변환본과 일치 (E4)
- 피드백 반영: E1 → 0-2절 3단 구조 팀 규약(합의 내용은 함태훈 notes/07 §6에도 기재), E6 BOM 조건, D1 → D2에 "베이스 collision 없음" 대안 병기, H3 → 공용 스크립트
- 4주차 참고: `checklist_check.py`는 2링크 파일 경로가 고정이고 `from_xml_path`를 써서, Windows 한글 경로에서는 [mjio.py](../../../members/park-jongjin/week03/scripts/mjio.py) 방식(문자열 + assets)이 필요. 우리 로봇 v0용으로 경로 인자화 예정
- C1 비고: 평면 3-DOF(C3) 채택은 팀장 결정 대기 (함태훈 §6 C1)

## 실습 PC 배정 후 (Isaac Sim)

- 액추에이터 대응: MuJoCo position `kp` → Drive stiffness, `kv` → Drive damping, `armature` → joint armature ([함태훈 대응표](https://github.com/taehoonham0118-cell/-1-/blob/main/notes/05_urdf_mjcf_mapping.md) §5-4) — 단위(rad vs deg)와 Drive Type(Force/Acceleration) 확인은 [urdf-import-checklist.md](urdf-import-checklist.md) B4
- 같은 URDF를 Isaac Sim 5.1 URDF Importer로 임포트 → [urdf-import-checklist.md](urdf-import-checklist.md) C절, 중력토크 · 말단 위치를 F3과 비교

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0.1 | 2026-10-06 | 함태훈 대응표(notes/05) 반영 |
| v0.2 | 2026-10-06 | 함태훈 제안 8개 · 점검 결과(OK 30 · NG 1 · N-A 5), 3단 파일 구조 팀 규약 |
| v0.3 | 2026-10-13 | B6 balanceinertia 기본 off 확정(김이겸 10/8 회신), A3 원인 점검 순서 · 기록 규칙, A3 · B6 기록표 신설 |
| v0.3.1 | 2026-10-16 | C3 채택 · C3-1 Isaac 분해 비고(팀장 10/16), 대상 4-DOF |
| v0.4 | 2026-10-16 | v1 기준: 0-3절 빌드 규약(제안), E7-1 중력보상, E8 액추에이터 응답, F6 검증 5종, M절 좌우 미러 |
| v0.4.1 | 2026-10-18 | 0-3절 확정(함태훈 동의), E7-1 A · B 구분, 링크 관성 입력 = 함태훈 관절점 원점 파일 |

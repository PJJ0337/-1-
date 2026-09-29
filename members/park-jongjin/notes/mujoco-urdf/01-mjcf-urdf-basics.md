# MJCF · URDF 기초 (week02 공통 학습)

> 근거: [MuJoCo XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html), [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html), [ROS URDF XML](http://wiki.ros.org/urdf/XML), B1 2·3주차 커리큘럼(김이겸).
> 실험 재현: [week02/scripts/oral_experiments.py](../../week02/scripts/oral_experiments.py), [week02/scripts/urdf_check.py](../../week02/scripts/urdf_check.py) (MuJoCo 3.14.0)

## 1. MJCF 구조 (화)

| 요소 | 역할 | 비고 |
|---|---|---|
| `<compiler>` | 단위 · 경로 · 관성 추정 규칙 | `angle="radian"`(G1), `inertiafromgeom` 기본 `auto` |
| `<option>` | timestep, integrator, 중력 | 기본 timestep 0.002 s, 중력 (0, 0, −9.81) |
| `<default>` | 클래스별 기본 속성 | G1: `class="g1"` 에 joint armature · position kp 공통 지정 |
| `<worldbody>` | body 트리의 루트 | |
| `<body pos quat>` | 강체. **부모 body 기준** 위치 · 자세 | 중첩 = 트리 |
| `<joint>` | 부모 ↔ 이 body 사이 자유도. `pos` · `axis`는 **이 body 좌표계** | hinge / slide / ball / free |
| `<geom>` | 형상 (충돌 · 시각화 · 질량 추정) | density 기본 1000 kg/m³ |
| `<inertial>` | 질량 · COM · 관성 | 없으면 geom에서 추정 |
| `<site>` | 질량 없는 표식 (센서 · 목표점) | |
| `<actuator>` | motor / position / velocity … | `gear` 기본 1, `ctrlrange` |

단위: 길이 m, 질량 kg, 각도는 `compiler angle` 에 따름 (**MJCF 기본은 degree**, URDF는 항상 rad → G1은 radian으로 명시).

## 2. URDF 구조 (수)

| 요소 | 역할 |
|---|---|
| `<link>` | 강체: `<inertial>`(origin · mass · inertia), `<visual>`, `<collision>` |
| `<joint type>` | `revolute`(범위 있음) · `continuous`(무한 회전) · `prismatic` · `fixed` · `floating` · `planar` |
| `<parent>/<child>` | 트리 연결. 부모가 없는 링크 1개 = 루트 |
| `<origin xyz rpy>` | **부모 링크 좌표계** 기준 자식 링크(=joint) 좌표계 위치 · 자세 |
| `<axis>` | **joint(자식 링크) 좌표계** 기준 회전/이동축 |
| `<limit>` | lower/upper [rad], effort [Nm], velocity [rad/s] — revolute · prismatic 필수 |
| `<dynamics>` | damping, friction |

## 3. URDF vs MJCF (목)

| 항목 | URDF | MJCF |
|---|---|---|
| 구조 표현 | link · joint를 **평면 나열**, parent/child로 연결 (트리만, 루프 불가) | body **중첩**으로 트리 표현, 루프는 `equality` 제약으로 |
| 관절 위치 | joint `origin`(부모 기준)이 곧 자식 링크 좌표계 | body `pos/quat` + joint `pos/axis`(body 기준) 두 단계 |
| 액추에이터 | 없음 (limit effort만) — 제어는 외부(ros2_control 등) | `<actuator>`로 모델 안에 정의 |
| 관성 | 링크마다 `<inertial>` 직접 지정 | `<inertial>` 또는 geom 밀도에서 자동 계산 |
| 시뮬 설정 | 없음 | `<option>` 으로 timestep · 적분기 · 접촉 파라미터 |
| MuJoCo 로딩 | 직접 읽음. **fixed joint 링크는 기본으로 부모에 합쳐짐**(`fusestatic`) | – |

## 4. 실습에서 확인한 것

### 2링크 팔 URDF → MuJoCo ([week02/models/two_link_arm.urdf](../../week02/models/two_link_arm.urdf))

| 확인 | 결과 |
|---|---|
| 컴파일 | 성공. 기본 로드 시 body = world · link1 · link2 (**base_link · tool0 사라짐**) |
| `<mujoco><compiler fusestatic="false"/></mujoco>` 삽입 | base_link · tool0 유지 (body 5개) |
| 질량 · 관성 | URDF 값과 일치 (link1 1.0 kg, Ixx 0.007725) |
| 수평 자세 중력 토크 | 해석해 shoulder 3.9731 Nm, elbow 0.7358 Nm = MuJoCo `qfrc_bias` 크기와 일치 (부호 반대: bias는 좌변 항) |
| tool0 위치 | (0.55, 0, 0.10) = 0.30 + 0.25, 베이스 높이 0.10 ✔ |

→ **fixed joint로만 붙은 링크(툴 · 센서 프레임)는 MuJoCo 기본 로드에서 사라진다.** Isaac Sim의 merge fixed joints와 같은 문제 → 3주차 임포트 체크리스트 항목.

## 5. 구술 답변 (week02)

### Q1. MJCF에서 body와 joint의 부모-자식 관계가 관절 좌표계를 어떻게 정하는가?

1. 자식 body의 `pos` · `quat`은 **부모 body 좌표계** 기준 → 자식 body 좌표계가 정해진다.
2. 그 body 안의 `<joint>` `pos` · `axis`는 **자식 body 좌표계** 기준으로 해석된다. 즉 관절 좌표계 = 자식 body 좌표계 + joint pos 오프셋.
3. joint는 "부모 ↔ 자식" 사이 상대 운동을 정의하며, joint가 없으면 자식은 부모에 고정.

실험(oral_experiments.py): 같은 `axis="0 1 0"` 이라도
- child body 회전 없음 → world 축 (0, 1, 0)
- child body를 z축 +90° 회전(`quat="0.7071 0 0 0.7071"`) → world 축 **(−1, 0, 0)**
- joint `pos="0.1 0 0"` → 회전축은 그대로, 앵커만 (0.4, 0, 1) / 회전 시 (0.3, 0.1, 1)로 이동

URDF와의 차이: URDF는 joint `origin`이 곧 자식 링크 좌표계라 관절 위치 = 링크 원점. MJCF는 body 원점과 관절 위치를 분리할 수 있다.

### Q2. inertial 태그를 생략하면 MuJoCo는 어떻게 처리하는가?

`compiler inertiafromgeom` (기본 `auto`) 규칙에 따른다.

| 경우 | 결과 (실험) |
|---|---|
| inertial 없음 + geom 있음 (auto) | geom 부피 × 밀도(기본 1000 kg/m³)로 질량 · 관성 계산. 0.2×0.1×0.1 m 상자 → **2.0 kg**, I = (0.0033, 0.0083, 0.0083) |
| geom `density="500"` | 1.0 kg |
| inertial 있음 + geom (auto) | **inertial 우선** (2 kg, 0.01) |
| `inertiafromgeom="true"` | inertial을 무시하고 geom으로 덮어씀 |
| `inertiafromgeom="false"` + inertial 없음 | 컴파일 오류 |
| 움직이는 body에 geom도 inertial도 없음 / visual geom(`density=0`)만 있음 | 컴파일 오류 `mass and inertia of moving bodies must be larger than mjMINVAL` |
| joint 없는(고정) body에 아무것도 없음 | 허용 (질량 0) |
| URDF link에 `<inertial>` 없음 | collision 형상으로 추정 (0.2×0.1×0.1 상자 → 2.0 kg) |

B1 시사점: A팀 관성값이 누락돼도 MuJoCo는 **오류 없이 밀도 1000으로 추정**해 버려 실제(알루미늄 2700, 속 빈 구조 등)와 크게 다를 수 있다 → ICD에서 관성 누락 금지, 모델 검증 시 링크별 질량 합계를 확인.

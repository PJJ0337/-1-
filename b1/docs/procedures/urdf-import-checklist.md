# 우리 로봇 URDF → Isaac Sim 임포트 체크리스트 (초안 v0)

| 항목 | 내용 |
|---|---|
| 상태 | **보류 — 실습 PC 배정 후 사용** (10/6 커리큘럼 변경, 현재는 [mujoco-urdf-loading-checklist.md](mujoco-urdf-loading-checklist.md)). **초안** (week03, 박종진) — 함태훈 URDF ↔ USD 대응표 검토 반영 전 · 실제 Isaac Sim 임포트(4주차) 전 |
| 대상 | Isaac Sim 5.1 URDF Importer (`isaacsim.asset.importer.urdf`), Isaac Lab 2.3.2 `UrdfConverterCfg` |
| 근거 문서 | [Isaac Sim 5.1 URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html) · [Omniverse URDF Importer 옵션](https://docs.omniverse.nvidia.com/kit/docs/omniverse-urdf-importer/latest/index.html) · [Isaac Lab urdf_converter](https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab/sim/converters/urdf_converter.html) · [Isaac Lab PR #4000](https://github.com/isaac-sim/IsaacLab/pull/4000) · [OpenUSD Physics](https://openusd.org/release/api/usd_physics_page_front.html) · [Omni Physics Articulations](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html) |
| 실험 근거 | [week02/scripts/urdf_check.py](../../../members/park-jongjin/week02/scripts/urdf_check.py), [oral_experiments.py](../../../members/park-jongjin/week02/scripts/oral_experiments.py), [week03/scripts/check_usd_vs_urdf.py](../../../members/park-jongjin/week03/scripts/check_usd_vs_urdf.py) |

## A. 임포트 전 — URDF 자체 점검

| No. | 점검 | 방법 · 기준 | 안 하면 생기는 일 (근거) |
|---|---|---|---|
| A1 | 움직이는 모든 link에 `<inertial>`(mass · origin · inertia 6성분) | `grep -c "<inertial"` ≥ 움직이는 링크 수 | MuJoCo는 collision 형상 × 1000 kg/m³로 **조용히 추정**(실험: 0.2×0.1×0.1 상자 → 2.0 kg). 구 Isaac 임포터 문서: 관성 누락 시 단위행렬 사용. 두 시뮬 결과 불일치 |
| A2 | 가상(dummy) 링크도 작은 질량 · 관성 부여 (예: 베이스 평면 3-DOF용 x/y 링크) | mass ≥ 1e-3 kg 수준 | MuJoCo: `mass and inertia of moving bodies must be larger than mjMINVAL` 컴파일 오류(실험 F·G·H). PhysX도 질량 0 강체 불가 |
| A3 | 단위: 길이 m, 각도 rad, 질량 kg | 메시 스케일 확인 (mm STL이면 `scale="0.001 0.001 0.001"`) | 1000배 크기 로봇 |
| A4 | 모든 revolute에 `<limit lower upper effort velocity>` | effort = 구동기 최대 토크 | Isaac Lab ActuatorCfg `effort_limit` 기본값 출처가 사라짐 |
| A5 | 메시 경로: `package://` 대신 URDF 기준 **상대 경로** | 다른 PC에서 임포트 시험 | 메시 누락 |
| A6 | 센서 · 툴 프레임은 **fixed joint로 붙은 전용 링크** (`imu_base_link` 등) | ICD 4절 이름 규칙 | Isaac Lab SensorCfg · FrameTransformer가 prim path로 지정 불가 |
| A6-2 | 인접 링크(부모-자식)의 collision 형상이 관절 근처에서 겹치지 않음 | 영점 자세 + 관절 범위 양 끝에서 접촉 수 확인 | MuJoCo는 월드 고정 부모-자식 접촉을 거르지 않아 관절이 막힘 (2링크 실험: 어깨가 수평에서 정지). Isaac은 Self-Collision off면 영향 없어 **두 시뮬 결과가 달라짐** |
| A7 | 폐루프 없음 (트리) | URDF는 루프 표현 불가 | Articulation은 링크당 inbound joint 1개, 루프는 끊어야 함 |
| A8 | MuJoCo로 먼저 로드해 사전 검증: 총질량, FK(말단 위치), 수평 자세 중력 토크 | `urdf_check.py` 방식 (2링크: 해석해 3.9731 Nm = MuJoCo) | 오류를 Isaac Sim(GPU)까지 가서 발견 |

## B. 임포터 옵션 (Isaac Sim 5.1 UI 기준)

| No. | 옵션 | 기본값 | 우리 로봇 권장 | 옵션별 결과 · 근거 |
|---|---|---|---|---|
| B1 | **Base Type** (Moveable / Static) | – | 팔 단독 검증: **Static** / 전신: **Moveable** | Static은 `root_joint`를 만들어 base link를 월드에 고정 (5.1 문서). Moveable은 바퀴 로봇용 = floating base |
| B2 | Default Density | – | **0** (URDF 질량 사용) | 질량이 없는 링크에만 적용. 0이면 물리엔진 기본 계산 (5.1 문서) → A1을 지키면 영향 없음 |
| B3 | Joint Configuration: Stiffness / **Natural Frequency** | – | 초기 형상 확인: Stiffness, 제어 튜닝: Natural Frequency | Natural Frequency: `Kp = m·ωn²`, `Kd = 2·m·ζ·ωn` (m = 관절 등가 관성) — 링크 관성이 바뀌어도 응답 속도 유지 |
| B4 | **Drive Type**: Acceleration / Force | **Acceleration** | **Force** (실기 토크 비교 · WBC) | Acceleration은 관성을 정규화해 질량 변화에 무관, Force는 토크를 스프링-댐퍼로 직접 적용 (5.1 문서). 게인 값의 의미가 달라지므로 A2 구동기 토크와 비교하려면 Force |
| B5 | Target Type: None / Position / Velocity | – | 팔: Position, 베이스 평면 관절: Velocity | Position은 stiffness, Velocity는 damping이 유효 (5.1 문서) |
| B6 | Ignore Mimic | 해제 | 해제 (핸드 선정 후 재검토) | 그리퍼 연동 관절 |
| B7 | Collision From Visuals | 해제 | 해제 (collision을 URDF에 직접) | collision이 없을 때 visual 메시로 생성 (5.1 문서) |
| B8 | Collider Type: Convex Hull / Convex Decomposition | – | 팔 링크: Convex Hull, 오목한 몸통: Decomposition | Hull 1개 vs 여러 개로 형상 근사 (5.1 문서) |
| B9 | Allow Self-Collision | 해제 | **해제**, 필요한 쌍만 나중에 | 관절부에서 collision 메시가 겹치면 불안정 (문서 경고) |
| B10 | Replace Cylinders with Capsules | – | 필요 시 | 원통 충돌체 → 캡슐 |
| B11 | **Merge Fixed Joints** | 5.1 UI에서 제거됨 | **사용하지 않음** (센서 프레임 보존) | 구 문서: "fixed joint로 연결된 링크를 합쳐 움직이는 관절에만 articulation 적용". **Isaac Sim 5.1 최신 임포터에서 merge-joints 지원 제거** → Isaac Lab은 importer 2.4.31로 고정해 `merge_fixed_joints` 유지 (PR #4000). MuJoCo `fusestatic`(URDF 기본 true)도 같은 동작: 2링크 팔의 base_link · tool0이 사라짐(실험) |
| B12 | Import 방식: Stage / Referenced Model | – | Referenced Model (로봇 USD를 별도 파일로) | 장면과 로봇 분리 → 로봇 모델만 교체 가능 |

## C. 임포트 후 — USD 점검

| No. | 점검 | 기준 |
|---|---|---|
| C1 | **ArticulationRoot 1개**, 위치 확인 | 고정 베이스: 월드 ↔ base의 fixed joint(또는 그 조상) / floating: 루트 링크(또는 조상). 중첩 불가 (OpenUSD Physics, Omni Physics) |
| C2 | 관절 수 · 이름 = URDF | `check_usd_vs_urdf.py` 방식 자동 비교 |
| C3 | 관절 범위: **USD는 degree** | URDF ±1.5708 rad ↔ USD ±90° (2링크 검증 통과) |
| C4 | 각 관절 body0·localPos0 = body1·localPos1 (world) | 불일치 시 시뮬 시작 시 링크가 튐 |
| C5 | 링크 질량 합계 = URDF 합계 = 목표 ≤ 80 kg | MassAPI의 명시 mass가 density 추정보다 우선 (OpenUSD) |
| C6 | Drive: maxForce = URDF effort, stiffness/damping 값 · 단위(토크/deg) | |
| C7 | 센서 프레임 링크(A6) prim이 존재 | Merge 사용 시 사라짐 (B11) |
| C8 | Isaac Lab: `ArticulationCfg` 로 로드 → 기본 자세 · 중력 하 정지 확인, `ActuatorCfg`의 `joint_names_expr`가 팔/베이스 그룹을 정확히 잡는지 | 액추에이터 그룹 분리 (Core Concepts) |
| C9 | MuJoCo 결과와 비교: 같은 자세의 중력 토크 · 말단 위치 | 두 시뮬 불일치 = 변환 오류 |

## 확인 필요 (4주차 실제 임포트 시)

- [ ] Isaac Sim 5.1 기본 임포터에서 fixed joint 링크가 실제로 어떻게 처리되는지 (링크 유지 여부)
- [ ] URDF `planar` joint 지원 여부 → 미지원 시 베이스를 prismatic x · prismatic y · revolute z + dummy 링크(A2)로 구성
- [ ] URDF `<dynamics damping friction>` 이 USD 어디로 가는지 (함태훈 대응표와 교차 확인)
- [ ] 임포트 결과 링크 prim이 중첩 계층인지 평탄 구조인지

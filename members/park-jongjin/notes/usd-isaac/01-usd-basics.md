# USD · USD Physics · Isaac Sim 개념 (week03 공통 학습)

> 근거: [OpenUSD 소개](https://openusd.org/release/intro.html), [USD Physics 스키마](https://openusd.org/release/api/usd_physics_page_front.html), [Omni Physics Articulations](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html), [Isaac Sim 5.1 URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html), B1 2·3주차 커리큘럼(김이겸).
> 실습 재현: [week03/scripts/](../../week03/scripts/) (usd-core 26.x, Isaac Sim 없이)

## 1. USD 기본 (월)

| 개념 | 설명 | 실습 파일에서 |
|---|---|---|
| Stage | 합성된 장면 전체. 루트 레이어 + 합성된 레이어들 | `Usd.Stage.CreateNew("cube.usda")` |
| Layer | 파일 1개 (.usda 텍스트 / .usd · .usdc 바이너리) | `cube.usda` |
| Prim | 경로로 식별되는 노드 (`/World/Cube`), 타입(Xform · Cube · Mesh …) | `def Cube "Cube"` |
| Attribute | 값 (`size`, `xformOp:translate`) | `double size = 0.2` |
| Metadata | 레이어 · prim 부가정보 | `upAxis = "Z"`, `metersPerUnit = 1`, `defaultPrim` |

**왜 Isaac Sim은 URDF가 아니라 USD를 쓰는가:** URDF는 로봇 1대의 기구학 · 관성만 담는다. USD는 로봇 · 환경 · 조명 · 센서 · 물리 설정을 한 장면으로 **레이어 합성**하고, 물리(USD Physics)와 렌더링(RTX)이 같은 데이터를 쓰며, 수천 개 환경 복제(인스턴싱)가 가능하다. 그래서 URDF는 **임포트해서 USD로 변환**해 쓴다.

## 2. 합성 (화)

| 방식 | 의미 | 실습 |
|---|---|---|
| **sublayer** | 레이어를 **통째로 겹침**. 위 레이어의 의견(opinion)이 아래보다 강함. prim 경로가 그대로 유지 | `two_link_arm_physics.usda` 가 `two_link_arm_geom.usda` 를 sublayer로 깔고 `over` 로 물리 속성만 덧씀 |
| **reference** | 다른 파일의 prim(기본: defaultPrim)을 **내 경로 아래로 가져옴**. 경로가 바뀜 | `scene.usda` 의 `/World/two_link_arm` → `two_link_arm_physics.usda` 의 `/two_link_arm` |
| variant | 한 prim 안에 선택지(예: 충돌체 정밀/단순) | 미사용 |
| `def` vs `over` | def = 정의, over = 이미 있는 prim에 의견만 추가 | 물리 레이어는 전부 `over` |

- 업축: Isaac Sim은 **Z-up**, 1 unit = 1 m (`metersPerUnit=1`), 질량 kg (`kilogramsPerUnit=1`).
- Xform 계층: 자식의 `xformOp:translate` 는 부모 기준 → URDF joint origin과 같은 역할.

## 3. USD Physics (수)

| 스키마 | 역할 | URDF 대응 |
|---|---|---|
| `PhysicsRigidBodyAPI` | 강체로 시뮬. 하위 prim은 같은 강체의 일부, 단 **중첩된 RigidBody는 독립적으로** 움직임 | link |
| `PhysicsMassAPI` | mass · centerOfMass · diagonalInertia · principalAxes. **명시 mass > density 추정**, 기본 밀도 1000 kg/m³ | inertial |
| `PhysicsCollisionAPI` | 충돌 형상 | collision |
| `PhysicsRevoluteJoint` | body0 · body1 관계 + localPos0/localRot0 · localPos1/localRot1, axis(X/Y/Z), **limit 단위 degree** | joint (revolute) |
| `PhysicsFixedJoint` | 고정. body0을 비우면 **월드에 고정** | fixed / 고정 베이스 |
| `PhysicsDriveAPI:angular` | `stiffness·(targetPos − p) + damping·(targetVel − v)`, maxForce, type(force/acceleration) | limit effort (+ 제어기) |
| `PhysicsArticulationRootAPI` | 하위 관절을 **축소 좌표(reduced coordinate)** 로 시뮬 | (해당 없음) |

관절 연결은 **prim 계층이 아니라 joint의 body0/body1 관계**로 정해진다. (2링크 USD는 Xform 계층으로 만들었지만, 각 링크의 RigidBody는 독립이고 joint가 묶는다.)

### 실습 결과 ([check_usd_vs_urdf.py](../../week03/scripts/check_usd_vs_urdf.py) — 모두 통과)

- RigidBody 3 · ArticulationRoot 1(월드 ↔ base_link fixed joint에) · PhysicsScene 1
- 링크 질량 URDF와 일치, 총 3.6 kg
- shoulder · elbow: body0·localPos0 = body1·localPos1 (world (0,0,0.1), (0.3,0,0.1)), axis Y, 범위 ±90° / ±135° = URDF ±1.5708 / ±2.3562 rad, maxForce 40 / 20 = URDF effort
- tool0 = (0.55, 0, 0.10) = URDF FK

## 4. Isaac Sim 개념 (목)

- **Articulation**: 관절로 연결된 강체 트리를 축소 좌표(루트 자세 + 관절각)로 푸는 구조. 링크당 inbound joint 1개, 루프는 끊어야 함.
- **Joint Drive**: stiffness(위치 게인) · damping(속도 게인). Position 타깃이면 stiffness, Velocity 타깃이면 damping이 주로 작동. Drive type이 Acceleration이면 관성으로 정규화된 게인.
- **URDF Importer 옵션**: base 고정 여부, 드라이브 설정(Natural Frequency: Kp = m·ωn², Kd = 2·m·ζ·ωn), collider, self-collision, (구버전) merge fixed joints → [임포트 체크리스트](../../../../b1/docs/procedures/urdf-import-checklist.md)
- 파이프라인 그림: [week03/img/pipeline.png](../../week03/img/pipeline.png)

## 5. 구술 답변 (week03)

### Q1. Isaac Sim에서 Articulation Root 위치가 왜 중요한가?

1. **고정 베이스인지 떠 있는 베이스인지를 결정**한다. 고정: 월드와 base를 잇는 fixed joint(또는 그 조상 prim)에 둔다. floating: 루트 링크(또는 그 조상)에 둔다. (OpenUSD Physics, Omni Physics)
2. Articulation은 **루트 기준 축소 좌표**(루트 자세 + 관절각)로 계산되므로, 루트가 곧 좌표의 기준이다. 자세 · 속도는 루트에만 설정 가능하고 비루트 링크에 설정하면 경고.
3. Articulation Root는 **중첩할 수 없다** (중첩 시 오류). 잘못 두면 로봇이 여러 articulation으로 쪼개지거나 관절이 최대 좌표 조인트로 풀려 정확도 · 성능이 떨어진다.
4. 우리 로봇: 팔 단독 검증 = 고정(Static base, root_joint), 전신 = base_link를 루트로 한 floating(또는 평면 3-DOF 가상 관절). Isaac Lab의 base 자세 관측 · WBC의 부동 자유도도 이 루트를 기준으로 한다.

실습: 2링크 USD에서 `root_joint`(body0 비움 = 월드, body1 = base_link)에 ArticulationRootAPI 적용 → 체크 스크립트로 1개임을 확인.

### Q2. merge fixed joints 옵션을 켜면 무엇이 사라지는가?

1. **fixed joint로 붙은 자식 링크 prim(과 그 좌표계)** 이 부모 링크에 합쳐져 사라진다: 툴 프레임, IMU · 카메라 장착 링크, 장식 커버 등. fixed joint 자체도 없어진다.
2. 사라진 링크의 **질량 · 관성은 부모 링크에 합산**되고 visual · collision 형상은 부모 밑으로 옮겨진다 → 물리적으로는 같지만 이름으로 참조할 수 없다.
3. 결과: Isaac Lab에서 그 링크 이름으로 `FrameTransformer` · 센서 · `body_names`를 지정하면 찾지 못한다. articulation 링크 수가 줄어 계산은 가벼워진다(움직이는 관절에만 articulation 적용이 옵션의 목적).
4. 버전 주의: **Isaac Sim 5.1 최신 URDF 임포터는 merge-joints 지원을 제거**했고, Isaac Lab은 이를 유지하려고 importer 2.4.31로 고정했다 (Isaac Lab PR #4000).

실습 근거: MuJoCo의 같은 기능(`fusestatic`, URDF 로드 시 기본 true)으로 2링크 팔을 로드하면 body가 5개 → **3개**(base_link는 world에, tool0은 link2에 합쳐짐). `fusestatic="false"` 로 다시 보존.

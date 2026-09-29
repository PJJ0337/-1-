# Isaac Lab Core Concepts 학습 정리

> 출처: 김이겸 대학원생 작성 Notion「캡스톤디자인(1)」— Isaac Lab 기초 / Core Concepts 9개 페이지, 원문 [Isaac Lab Core Concepts v2.1.0](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/index.html).
> 학부생 박종진 학습 정리(1주차). 각 절 끝의 **B1 적용 지점**은 본인 정리.
> 설치 대상은 v2.3.2이므로 Cfg 인자명은 2.3.x 문서로 재확인할 것.

목차
1. [Task Design Workflows](#1-task-design-workflows)
2. [Actuators](#2-actuators)
3. [Sensors (공통)](#3-sensors-공통)
4. [Camera](#4-camera) · [Contact Sensor](#5-contact-sensor) · [Frame Transformer](#6-frame-transformer) · [IMU](#7-imu) · [Ray Caster](#8-ray-caster)
5. [Motion Generators](#9-motion-generators)

---

## 1. Task Design Workflows

원문: [task_workflows](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/task_workflows.html)

RL 태스크 흐름: **환경 설정 → 리셋 → 스텝(행동 적용 · 상태 갱신 · 보상 · 종료 판정) → 렌더링 → 에피소드 종료/전환**

| 항목 | Manager-Based | Direct |
|---|---|---|
| 구조 | 관측 · 행동 · 보상 · 이벤트 등을 **매니저**로 분리, 각각 설정 클래스(Cfg) | 한 클래스에서 전부 직접 구현 |
| 설정 | `@configclass` 로 항목 정의 (예: `RewTerm`) | `_get_rewards()` 등 메서드 직접 작성 |
| 재사용 · 확장성 | 높음 | 낮음 |
| 복잡도 | 초기 설정이 다소 복잡 | 단순 · 빠른 프로토타입 |

Manager-Based 보상 정의 예 (Cartpole):

```python
@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    pole_pos = RewTerm(func=mdp.joint_pos_target_l2, weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]), "target": 0.0})
    cart_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"])})
    pole_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"])})
```

**B1 적용 지점:** URDF 모델 검증 환경은 Manager-Based가 적합. `ArticulationCfg` · `ActuatorCfg` · `SensorCfg` 만 교체해 관절 · 구동기 · 센서가 의도대로 잡히는지 점검할 수 있고, 문제 항목을 격리하기 쉽다.

---

## 2. Actuators

원문: [actuators](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/actuators.html)

- 실제 구동기는 지연 · 최대 속도/토크 제한 · 비선형 반응을 가진다.
- 시뮬레이션 제어 방식: 위치 · 속도 제어(물리 엔진 내부 PD가 토크 계산), 토크 제어(사용자가 직접 지정 — 이상적 드라이브 가정).

| 모델 | 설명 |
|---|---|
| 암시적(Implicit) | 물리 엔진이 제공하는 이상적 PD. 지연 · 포화 · 마찰 미반영 |
| 명시적(Explicit) | 사용자 구현. ① 목표 추종 토크 계산 → ② 모터 성능으로 클리핑 → ③ 적용 |

명시적 모델 예 — `IdealPDActuator`:

```
τ_computed = k_p (q_des − q) + k_d (q̇_des − q̇) + τ_ff
τ_applied  = clip(τ_computed, −τ_max, τ_max)
```

- 액추에이터 모델은 입력(목표 명령) → 출력(적용 명령)만 계산하는 블록이며, 어느 관절인지는 모른다. 관절에 적용하는 것은 `isaaclab.assets.Articulation` 의 역할.
- **액추에이터 그룹**: 같은 모델을 쓰는 관절끼리 묶는다. (예: 다족 이동 조작기는 다리 그룹 / 팔 그룹 분리)

**B1 적용 지점:** 우리 로봇은 팔(6-DOF × 2)과 홀로노믹 베이스의 구동기가 달라 **그룹을 분리**해야 한다. 암시적 모델은 실기 토크를 과대평가하므로 WBC 검증 전에는 실측 모터 사양(τ_max, 속도 한계)을 넣은 명시적 모델이 필요 → A팀에 모터 사양 제공 형식 요청(ICD).

---

## 3. Sensors (공통)

원문: [sensors](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/index.html)

- 모든 센서는 `SensorBase` 상속. 측정값 = 레이캐스트 결과 · 렌더 이미지 · 시뮬레이터 ground-truth 등.
- `update_period`(시뮬레이션 시간 기준)마다 갱신. 복제 환경 전체 버퍼를 **벡터화**해 관리.
- 갱신 흐름: 매 dt마다 누적 시간 += dt → `update_period` 이상이면 플래그 → 다음 조회 시 `_update_buffers_impl()` 로 실제 갱신.

## 4. Camera

원문: [camera](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/camera.html)

- `render_product` 로 정의. Annotator로 RGB/RGBA, Depth, Semantic/Instance(ID) Segmentation, Normals `(B,H,W,3)`, Motion Vectors `(B,H,W,2)` 등 출력.
- **대역폭:** 800×600 · 32-bit ≈ **2 MB/장**, 60 fps ≈ **120 MB/s** — 카메라 × 환경 수만큼 증가 → 병목.
- **Tiled Rendering** (Isaac Sim ≥ 4.2, `TiledCamera` / `TiledCameraCfg`): 64환경 × 84×84 → **672×672 한 장**으로 묶어 동기화 1회. RTX 4090급에서 512 카메라 권장.

## 5. Contact Sensor

원문: [contact_sensor](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/contact_sensor.html)

- 지정 강체에 작용하는 **순 접촉력** 반환. 기본은 모든 접촉 합산, 필터링은 **다대일(many-to-one)** 만 지원.
- 다족 로봇: 발마다 1개. 손끝이 하나의 강체면 1개로 충분.

## 6. Frame Transformer

원문: [frame_transformer](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/frame_transformer.html)

- 소스 프레임(USD prim path 1개) 기준 타겟 프레임들(regex 가능)의 상대 위치 · 회전을 벡터화해 계산.

## 7. IMU

원문: [imu](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/imu.html)

- 선형 가속도 · 각속도 측정. 실제 센서처럼 **+g 중력 보정이 기본 포함** → 정지 시 중력 반대 방향 +g 출력.

## 8. Ray Caster

원문: [ray_caster](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/ray_caster.html)

- 방향 지정 광선의 최초 충돌점 반환(반사 · 재질 무시). Warp GPU 가속.
- 대상 mesh를 미리 지정해야 하며 **정적 mesh만** 지원. 구성: 패턴(LiDAR · grid), parent xform, offset.

**B1 적용 지점 (센서):** URDF에 센서 장착 링크(IMU · 카메라 프레임)를 **별도 링크/fixed joint로 명시**해 두어야 Isaac Lab에서 prim path로 지정 가능 → URDF 임포트 시 merge fixed joints 옵션과 충돌 여부 확인 필요(3주차 체크리스트 항목).

---

## 9. Motion Generators

원문: [motion_generators](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/motion_generators.html)

- 작업은 작업공간(end-effector 궤적), 명령은 관절공간 → 두 제어 접근. 모델 오차를 흡수하려면 **컴플라이언스**(수동: 탄성 구동기 / 능동: 임피던스 · 하이브리드 힘/운동 제어).

| 분류 | 항목 |
|---|---|
| Joint-space | 토크 · 속도 제어, 고정/가변 강성 · 가변 임피던스 위치 제어 (`ActuatorControlCfg`, `JointImpedanceController`) |
| Task-space | Differential IK (`DifferentialInverseKinematics`), 임피던스, Operational-space, 폐루프 비례 힘, 하이브리드 힘-운동 |
| Reactive planner | **RMPFlow**(Lula, 가속도 기반 RMP 조합, 동적 충돌 회피), **MPC**(OCS2, SLQ 리시딩 호라이즌, 관절 한계 · 자기 충돌을 소프트 패널티로) — 현재 CPU 구현이라 학습 중 속도 저하 |

**B1 적용 지점:** 모바일 매니퓰레이터(홀로노믹 베이스 + 양팔)는 OCS2 MPC의 적용 대상에 해당. 모델(URDF)의 관절 한계 · 충돌 형상이 정확해야 제약 처리가 의미 있음.

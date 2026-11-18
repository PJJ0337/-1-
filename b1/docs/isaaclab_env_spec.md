# B1 Isaac Lab 환경 명세 v0.1 — 접촉 집기 과제 (Manager-Based) · MuJoCo 코드 1:1 대응

- 작성: 박종진 (week09, 2026-11-18) · 기준 버전: Isaac Sim 5.1 · Isaac Lab 2.3.2 (노션 개발환경, [import_settings.md](../models/v1/docs/import_settings.md)와 같음)
- 목적: 실습 PC 배정 전에 **MuJoCo 집기 시나리오(week08 `scenarios/`) · 데이터(week08 `b1-dataset-1.0`) · BC 평가(week09 `policies/`)** 의 각 요소를 Isaac Lab Manager-Based 환경의 어떤 term으로 옮길지 정해 둔다. 배정 주 수요일 과제 = 이 표대로 `ManagerBasedRLEnvCfg` 코드 작성.
- 같이 쓰는 파일: 로봇 [`b1_v1_cfg.py`](../models/v1/isaac/b1_v1_cfg.py)(week06) + **[`b1_pick_cfg.py`](../models/v1/isaac/b1_pick_cfg.py)(week09 신규: 손가락 URDF 사본 · 그리퍼 액추에이터 · 테이블 · 물체 · 접촉 센서)** ← 생성 `scripts/export_isaac_pick.py`
- ⚠ = Isaac 쪽 함수 · 인자 이름을 실습 PC 버전에서 확인할 항목. 이 문서의 이름은 Isaac Lab 2.x `isaaclab.envs.mdp` 기준
- **함태훈 담당 칸(대기)**: §6 Event 매핑의 「함태훈 randomize.py」 열 — push 받으면 이 표에 통합(v0.2)

## 1. 파일 · 개념 대응

| MuJoCo (우리 코드) | Isaac Lab | 비고 |
|---|---|---|
| `pick_env.build()` (b1.xml + 메모리 손가락 + 테이블 · 물체) | `InteractiveSceneCfg`: robot = `B1_PICK_CFG`, table · obj_box · obj_cyl = `RigidObjectCfg`, 접촉 센서 2개 | 손가락은 URDF 사본(`b1_v1_isaac_pick.urdf`)에 넣음 — 원본 URDF · 생성 MJCF는 그대로 |
| `pick_env.sample_task(seed)` · `reset()` | `EventTerm(mode="reset")` 묶음 (§6) | seed → 같은 초기 상태 규칙은 Isaac에서 `env.seed` + 이벤트 순서로 대체(비트 단위 같음은 보장 안 됨) |
| `expert.Expert` (상태기계 · 전신 IK · 궤적) | 환경 밖 **스크립트 정책**(`play_expert.py`) — 환경은 관측 · 행동만 제공 | 시연 수집 = Isaac Lab `RecorderManager` → HDF5. 필드 이름은 `b1-dataset-1.0`에 맞춰 후처리 |
| `expert._judge()` 성공 판정 | `TerminationTermCfg(func=b1_success)` + 같은 함수를 `info["success"]`로 | §7 |
| `expert.apply_action()` (action 8 → ctrl 13) | `ActionManager` 항 4개 (§5) | 오른팔 영점 고정 = 오른팔 액션 항 없음 + 기본 목표 0 |
| `policies/bc_common.features()` (39) | `ObservationGroupCfg("policy")` 항 12개 (§4) | 대부분 사용자 정의 함수 `mdp_b1.py` (§9) |
| `policies/rand_dyn.py` · 함태훈 `randomize.py` | `EventTerm(mode="startup" / "reset")` | §6 |
| `policies/eval_bc.py` (T_MAX 34 s, 판정 · 실패 분류) | `episode_length_s = 34`, termination 항, `extras["log"]` 실패 분류 | |

## 2. Scene · 시뮬레이션

| 항목 | MuJoCo | Isaac Lab | 확인 |
|---|---|---|---|
| 로봇 | v1 MJCF (a2_doc case), 베이스 slide · slide · hinge | `B1_PICK_CFG` (fix_base, prismatic x · y + continuous yaw, 팔 · 베이스 게인 week06) | import_settings.md §4 첫날 재현 |
| 손가락 | slide 2개 + equality 대칭 + position kp 3000 · ±40 N | prismatic 2개, **같은 목표를 두 관절에** (액션 항에서 1값 → 2관절), stiffness 3000 · effort 40 | mimic 미사용 ⚠ |
| 테이블 | box 0.6 × 1.8 × 0.70 m, 앞 가장자리 x 0.45 | `CuboidCfg` kinematic, 중심 (0.75, 0, 0.35) | |
| 물체 | 상자 60 mm · 원기둥 Ø56 × 65 mm, 0.2 ~ 3 kg | `CuboidCfg` · `CylinderCfg` (env마다 둘 다 생성, 과제 아닌 쪽은 바닥 멀리) | 물체 종류를 env마다 바꾸는 대안: `MultiAssetSpawnerCfg` ⚠ |
| 마찰 | 손가락 · 물체 1.0, 테이블 0.8, condim 4, 결합 = max | `RigidBodyMaterialCfg(static = dynamic = μ, combine "max")` | condim · solref · solimp 대응 없음 → 미끄러짐 표 재현으로 확인 |
| 시간 | dt 0.5 ms × 40 = 제어 50 Hz | `sim.dt = 0.002`, `decimation = 10` (50 Hz) 시작값 | 0.5 ms는 MuJoCo soft contact 발산 대책. PhysX는 그리퍼 미끄러짐 54조합(`gripper_slip.md`) 재현으로 dt 결정 ⚠ |
| 에피소드 | 전문가 23.9 ~ 28.6 s, BC 평가 34 s | `episode_length_s = 34.0` | |
| 카메라 | `overview` 96 × 96 (선택 관측) | `TiledCameraCfg` 같은 위치 · 자세 | 상태 기반 BC에는 안 씀 |

## 3. 좌표 규칙

- world = MuJoCo 월드 = Isaac env 원점(+x 전방, +z 위). 베이스 관절이 world 축 고정(slide · prismatic) → **로봇 좌표 = 베이스 x · y · yaw** (높이는 world 그대로) — 데이터 명세 §1과 같음.
- 관측 · 행동의 「로봇 좌표」 변환은 모두 `R(yaw)ᵀ` 하나. Isaac Lab 기본 `root_*`는 루트(odom, 고정)라 쓰면 안 됨 → 사용자 정의 함수에서 `base_yaw_link` 기준으로 계산 ⚠

## 4. Observation (group `policy`, 39차원 = `bc_common.FEAT_NAMES` 순서)

| # | MuJoCo 특징 | 차원 | Isaac Lab term | 비고 |
|---|---|---|---|---|
| 1 | base x · y · sin yaw · cos yaw | 4 | `b1.base_pose_sincos` (joint_pos of `base_[xy]_joint`, `base_yaw_joint`) | 사용자 정의 |
| 2 | 베이스 속도(로봇 좌표) vx · vy · wz | 3 | `b1.base_vel_body` (joint_vel 3개 → R(yaw)ᵀ) | 사용자 정의 |
| 3 | 왼팔 관절 4 | 4 | `mdp.joint_pos` (`SceneEntityCfg("robot", joint_names=["left_shoulder_.*","left_elbow_joint"])`) | `joint_pos_rel`은 기본 자세 차 — 영점 = 0이라 같음 |
| 4 | 왼팔 관절 속도 4 | 4 | `mdp.joint_vel` (같은 joint_names) | |
| 5 | 그리퍼 벌림(안쪽 면 거리) | 1 | `b1.gripper_width` = 손가락 2관절 위치 합 | |
| 6 | 끝점 위치(로봇 좌표) | 3 | `b1.ee_pos_base` ← `FrameTransformerCfg`(left_gripper_link, offset (0, 0, −0.1)) | ee = 손바닥 아래면 중심 |
| 7 | 끝점 x · z축(로봇 좌표) | 6 | `b1.ee_axes_base` (FrameTransformer 쿼터니언 → 회전행렬 열 0 · 2) | |
| 8 | 물체 위치(로봇 좌표) | 3 | `b1.obj_pos_base` (과제 물체 `RigidObject.data.root_pos_w`) | Lift 예제 `object_position_in_robot_root_frame`과 같은 역할 — 루트가 달라 새로 씀 |
| 9 | 물체 − 끝점 | 3 | `b1.obj_minus_ee_base` | |
| 10 | 물체 yaw(로봇 기준) sin 4ψ · cos 4ψ (상자), 0 (원기둥) | 2 | `b1.obj_yaw_sym` | 상자 90° 대칭 |
| 11 | 물체 종류 one-hot · 물체 z축 · ẑ | 3 | `b1.obj_kind_onehot`, `b1.obj_upright` | 종류는 env 버퍼(리셋 때 정함) |
| 12 | 목표 원 중심(로봇 좌표) | 3 | `b1.goal_pos_base` (고정 목표 — 나중에 `CommandTerm`으로 무작위화) | |
| 선택 | 직전 행동 / 경과 시간 | 8 / 1 | `mdp.last_action` / `b1.episode_time` | week09 BC 비교 조건(§10 결과) |

- 정규화: BC 체크포인트(`policies/ckpt/bc_*.pt`)의 평균 · 표준편차를 정책 쪽에서 적용 (Isaac obs 항은 원 단위 그대로). 노이즈 항(`noise=`)은 랜덤화 단계에서 함태훈 센서 노이즈 범위로.
- 명세 §2의 `obs/stage`는 관측에 넣지 않음(데이터와 같은 원칙).

## 5. Action (BC 출력 2종 + 재생용 1종)

| 이름 | 차원 | MuJoCo | Isaac Lab term | 비고 |
|---|---|---|---|---|
| 베이스 속도(로봇 좌표) | 3 | `action_base_body` → R(yaw) → slide 속도 명령 (velocity kv 200 · 50) | **`b1.BaseBodyVelocityAction`** (사용자 정의 ActionTerm: R(yaw)로 world 속도 → `set_joint_velocity_target`(base 3관절)) | Isaac 기본 `JointVelocityActionCfg`는 world 축 속도라 회전 변환이 없음. 실기 베이스 명령 = 로봇 좌표라 이 형식이 기준 |
| 팔 — 관절 출력(BC joint) | 4 | 목표 = 현재 관절 + 출력(잔차) | `mdp.RelativeJointPositionActionCfg(joint_names=[left 4], scale=1.0)` ⚠ | 「상대」 기준이 직전 목표인지 현재 위치인지 버전 확인 — 우리 학습은 **현재 위치 + 잔차** |
| 팔 — 끝점 출력(BC ee) | 6 | 끝점 목표 위치(로봇 좌표, 현재 대비) 3 + z축 3 → 팔 4관절 DLS(위치 3 + 축 3 최소제곱, 직전 해에서 시작) | `mdp.DifferentialInverseKinematicsActionCfg(body_name="left_gripper_link", body_offset=(0,0,−0.1), controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"))` ⚠ | 4-DOF라 pose 6조건 > 4 → 최소제곱. 축만 맞추는 가중은 Isaac DLS에 없음 → 사용자 정의 IK가 필요할 수 있음 |
| 팔 — 절대 관절 목표(재생) | 4 | 데이터 `action[3:7]` | `mdp.JointPositionActionCfg(use_default_offset=False)` | 시연 재생 · 전문가 |
| 그리퍼 벌림 | 1 | 0 ~ 0.055 m → 손가락 2개 같은 값 | **`b1.SymmetricGripperAction`** (1값 → `left_finger_[lr]_joint` 위치 목표) | RL로 갈 때는 `BinaryJointPositionActionCfg`(열기 · 닫기) 대안 |
| 오른팔 | 0 | 영점 고정 | 액션 항 없음, 기본 목표 0 유지 | |

## 6. Event (리셋 · 랜덤화)

| MuJoCo | 모드 | Isaac Lab term | 범위 · 근거 | 함태훈 randomize.py (대기) |
|---|---|---|---|---|
| `mj_resetData` + 팔 영점 · 그리퍼 벌림 | reset | `mdp.reset_scene_to_default` | — | |
| 과제 물체 종류 (상자 / 원기둥 ½) | reset | `b1.choose_object` (과제 아닌 물체는 (−1.5, ±1.5) 바닥으로) | — | |
| 물체 위치 · yaw | reset | `mdp.reset_root_state_uniform(pose_range={x: (0.51, 0.67), y: (0.0, 0.35), yaw: ±45°})` | SPAWN (S17) | |
| 물체 질량 0.2 ~ 3 kg 로그 균등 | reset | `mdp.randomize_rigid_body_mass(operation="abs", distribution="log_uniform", mass_distribution_params=(0.2, 3.0))` ⚠ | S17 · 토크 예산 3 kg | |
| 물체 관성(균일 밀도 비례) | reset | `recompute_inertia=True` ⚠ (없으면 사용자 정의) | | |
| 마찰 ×0.7 ~ 1.3 (손가락 · 물체 · 테이블 각각) | startup | `mdp.randomize_rigid_body_material(static/dynamic_friction_range, asset_cfg=…)` ×3 | S16 가정, 실측 전 ±30 % | 범위 · 민감도 |
| 파지력 30 ~ 50 N | reset | `mdp.randomize_actuator_gains`로는 안 됨 → `b1.randomize_effort_limit(actuator="gripper")` 사용자 정의 ⚠ | S15 ±25 % | |
| 팔 armature ×0.8 ~ 1.2, damping ×0.5 ~ 2 | startup | `mdp.randomize_joint_parameters(armature_distribution_params, operation="scale")` + damping은 `randomize_actuator_gains(damping_…)`로 근사 | S3 · S4 | |
| 팔 kp ×0.9 ~ 1.1 | startup | `mdp.randomize_actuator_gains(stiffness_distribution_params=(0.9, 1.1), operation="scale")` | S12 | |
| 베이스 하체 질량 ×0.85 ~ 1.15 | startup | `mdp.randomize_rigid_body_mass(body_names="base_link", operation="scale")` | S10 역산값 | |
| 베이스 kv ×0.8 ~ 1.2 | startup | `mdp.randomize_actuator_gains(actuator "base", damping_distribution_params=(0.8, 1.2))` | S12 · 함태훈 week05 τ | |
| (센서 노이즈 · 제어 지연) | — | obs `noise=` · `ActionTerm` 지연 버퍼 | **함태훈 범위** | 대기 |

- 위 범위 = week09 BC 평가용 임시판 `policies/rand_dyn.py`(seed → 같은 표본). 함태훈 `randomize.py` · 민감도 Top 3가 오면 같은 표로 교체하고 BC 「랜덤화 있음」을 다시 돌린다.

## 7. Termination

| MuJoCo (`expert._judge`, `eval_bc.run`) | Isaac Lab term | 성공/실패 |
|---|---|---|
| 34 s 경과 | `mdp.time_out` (`time_out=True`) | 실패(분류 = 끝 상태로 grasp_miss / place_miss / not_released …) |
| 목표 원(40 mm) 안 · 테이블 위 ±10 mm · 기울기 < 15° · 손가락이 물고 있지 않음 · **0.5 s 유지** · 한 번이라도 30 mm 들어 올림 | `b1.success_placed` (상태 버퍼로 유지 시간 · 들어 올림 기록) | 성공 |
| 로봇(손가락 제외) ↔ 테이블 접촉 | `mdp.illegal_contact(sensor_cfg=ROBOT_TABLE_CONTACT, threshold=1.0 N)` | 실패 table_collision |
| 물체가 테이블 아래로 0.1 m 이상 떨어짐 | `mdp.root_height_below_minimum(minimum_height=0.60, asset_cfg=obj)` | 실패 drop |
| MuJoCo 경고(BADQACC · BADQVEL · BADQPOS) | `b1.nan_guard` (상태에 NaN · 비정상 속도) | 실패 sim_unstable |

## 8. Reward (RL 단계용 초안 — 지금 BC · 전문가에는 쓰지 않음)

| 항 | 함수 | 가중(초안) |
|---|---|---|
| 끝점 → 물체 접근 | `1 − tanh(‖ee − obj_grasp‖ / 0.1)` | 1.0 |
| 파지 | 패드 2개 동시 접촉 (`PAD_OBJ_CONTACT`) | 2.0 |
| 들기 | 물체 높이 > 테이블 + 30 mm | 5.0 |
| 목표 접근(들고 있을 때) | `1 − tanh(‖obj − goal‖ / 0.2)` | 8.0 |
| 놓기 성공 | `success_placed` | 20.0 |
| 벌점 | 행동 변화율 `mdp.action_rate_l2`, 팔 관절 속도 `joint_vel_l2`, 테이블 접촉 | −1e-3 · −1e-4 · −5 |

## 9. 새로 써야 할 사용자 정의 함수 (`b1_tasks/mdp_b1.py`)

`base_pose_sincos`, `base_vel_body`, `gripper_width`, `ee_pos_base`, `ee_axes_base`, `obj_pos_base`, `obj_minus_ee_base`, `obj_yaw_sym`, `obj_kind_onehot`, `obj_upright`, `goal_pos_base`, `episode_time` (관측) · `BaseBodyVelocityAction`, `SymmetricGripperAction` (행동) · `choose_object`, `randomize_effort_limit` (이벤트) · `success_placed`, `nan_guard` (종료). 각 함수의 수식 = `policies/bc_common.py` · `policies/eval_bc.py`의 같은 이름 계산 — **MuJoCo 쪽이 기준(정답 값)**, Isaac 첫 구현은 같은 상태에서 39차원 특징 차를 출력해 비교(허용 1e-4).

## 10. 이식 후 확인 순서 (실습 PC 배정 주)

1. `export_isaac_pick.py` 산출물 임포트 → 손가락 2관절 · 한계 · 질량 확인, 그리퍼 미끄러짐 표(`gripper_slip.md` 54조합) 재현 → PhysX dt 결정
2. 전문가 시연 1개(`datasets/samples/ep_01001.hdf5`)를 절대 관절 목표 + 베이스 속도로 재생 → 물체 최종 위치 · 성공 판정 비교 (MuJoCo 재생 차 0 기준, Isaac은 성공 판정 일치로 대신 — 명세 §4)
3. 관측 39차원: 같은 qpos에서 MuJoCo `bc_common.features` vs Isaac obs 차
4. BC 체크포인트를 Isaac에서 롤아웃 → MuJoCo 성공률(§ week09 `policies/results/eval.md`)과 비교 = sim-to-sim 차
5. Event 표(§6)로 랜덤화 켜고 같은 평가

## 판 이력

| 판 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 11/18 | 박종진 초안: 1:1 대응표 · 관측 39 · 행동 3종 · 이벤트(임시 rand_dyn) · 종료 · 보상 초안, 집기 scene cfg 생성(`export_isaac_pick.py`) |
| 0.2 | (예정) | 함태훈 randomize.py EventTerm 매핑 통합, 센서 노이즈 · 지연 |

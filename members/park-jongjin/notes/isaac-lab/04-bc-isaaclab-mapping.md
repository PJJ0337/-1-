# 04. 행동 복제(BC) · 공변량 이동 · Isaac Lab Manager-Based 대응 (week09 구술 대비)

근거: [policies/](../../../../policies/) (bc_common · train_bc · eval_bc · collect_dart · rand_dyn) · [eval.md](../../../../policies/results/eval.md) · [isaaclab_env_spec.md](../../../../b1/docs/isaaclab_env_spec.md) · [b1_pick_cfg.py](../../../../b1/models/v1/isaac/b1_pick_cfg.py)

## 0. 이번 주 BC 구성 (한 줄씩)

- 데이터: 8주차 시연 100개(seed 1000 ~ 1099, 성공 100) → 학습 90 · 검증 10(**에피소드 단위** 분리 — 같은 에피소드의 이웃 프레임이 양쪽에 섞이면 검증 손실이 과하게 좋게 나옴)
- 입력 39 = 로봇(베이스) 좌표 특징: 베이스 pose · 속도, 팔 관절 · 속도, 그리퍼 폭, 끝점 위치 · x/z축, 물체 위치 · 물체−끝점 · 상자 yaw(4ψ) · 종류 · 기울기, 목표 위치. `stage`는 안 넣음(명세 §2 — 정답 단계를 주면 상태기계를 베끼는 것)
- 출력: 관절 = [베이스 속도(로봇 좌표) 3, 팔 목표 − 현재 4, 그리퍼 1] / 끝점 = [베이스 3, 끝점 목표 − 현재 3, 그리퍼 z축 3, 그리퍼 1] → 롤아웃에서 팔 DLS IK
- 모델: MLP 3 × 256 ReLU, AdamW 1e-3 · cosine, 40 epoch, 입력 · 출력 평균/표준편차 정규화(체크포인트에 같이 저장), CPU 30 ~ 300 s
- 평가: 처음 보는 seed 2000 ~ 2049, 전문가와 같은 판정 + 실패 분류, 랜덤화 없음/있음(같은 seed = 같은 표본)

## Q1. BC 정책이 전문가보다 성공률이 낮은 이유(공변량 이동)와 이를 줄이는 방법은

**이유 — 공변량 이동(covariate shift) · 오차 누적(compounding error)**
- BC는 「전문가가 방문한 상태 분포 d_π*」에서만 지도학습한다. 롤아웃 때는 정책 자신의 상태 분포 d_π̂를 돈다. 한 주기 오차 ε가 생기면 처음 보는 상태로 가고, 거기서는 라벨이 없어 오차가 더 커짐 → 길이 T 과제에서 최악 오차가 **O(εT²)**(Ross & Bagnell 2010; DAgger는 O(εT)). 우리 과제는 T ≈ 1,230주기(24.6 s × 50 Hz)로 길다.
- 실제로 본 것(우리 데이터): **검증 MSE는 0.0019(시연 프레임 기준 아주 잘 맞음)인데 롤아웃 성공 0 %** — open-loop로 시연 상태를 넣으면 베이스 속도 · 팔 잔차를 소수 셋째 자리까지 맞추지만, 닫힌 루프에서는
  ① 베이스가 경유점에 0.3 rad 못 미친 채 다음 단계로 넘어가 테이블 충돌(시연에는 「덜 돈 상태에서 마저 도는」 예가 없음)
  ② **멈춤 모호성**: 전문가는 구간 사이(경유점 도착 → 팔 자세 변경)에서 5차 궤적이라 속도 0에서 출발 → 「정지 상태」의 라벨이 거의 0 → BC가 그 자리에 멈춤(숨은 변수 = 궤적 시간, 관측에 없음)
  ③ 직전 행동을 입력에 넣으면 멈춤은 풀리지만 **copycat**(직전 행동을 베끼며 오차가 되먹임 → 파지 중 베이스가 계속 회전)
- 전문가는 같은 판정에서 49/50(98 %, 랜덤화 있음도 98 %): 실패 1 = 계획 단계 IK 실패(seed 2002). 전문가가 랜덤화에 강한 이유 = 측정 기반 단계 전환 + 피드백(베이스 Kp · 서보) + 계획은 기구학만 씀
- 관절 vs 끝점 출력: open-loop 오차는 같음(0.24 · 0.22 mm). 닫힌 루프에서 끝점 출력은 예측한 끝점 + 축(5조건)이 4-DOF 팔이 낼 수 있는 곡면에서 조금만 벗어나도 IK가 큰 관절 변화로 답함(잔차 최대 중앙값 75 mm) → 수치 발산 5건. 4-DOF에선 관절 출력이 맞음

**줄이는 방법 (해 본 것 · 개념)**

| 방법 | 원리 | 이번 주 결과 |
|---|---|---|
| 데이터 늘리기 · 다양화 | d_π* 넓힘 | (DART와 함께) |
| **DART** — 시연 중 실행 명령에 노이즈, 라벨은 깨끗한 전문가 명령 (Laskey 2017) | 「벗어난 상태 → 되돌아오는 행동」 예시를 만듦. 전문가가 피드백 제어(베이스 위치 Kp 2.0)라 라벨이 곧 복귀 행동 | 300 seed 수집 → 성공 148개(19.7만 프레임). 테이블 충돌 35 → 3, 대신 멈춤(못 집음 46) — 0 % |
| **액션 청킹** — 앞으로 K주기 명령을 한 번에 예측, 롤아웃은 시간 앙상블 (ACT, Zhao 2023) | 결정 횟수를 1/K로 줄여 누적 오차 감소 + 「앞으로 움직일 계획」이 들어 있어 멈춤 모호성 해소 | K 10(0.2 s) 2/10 → **K 25(0.5 s) 36 %**(랜덤화 24 %) · K 50 22 %(26 %) — 0.2 s로는 5차 궤적 출발을 못 봄, 1 s는 반응이 늦어 정밀 단계 오차 |
| DAgger — 정책을 돌리며 방문한 상태를 전문가에게 다시 물어 라벨 (Ross 2011) | d_π̂ 위에서 학습 → O(εT) | 개념만: 우리 전문가는 상태기계 · 계획 이력(경유점 · 계획 해)을 가져서 임의 상태에 「지금 뭘 해야 하나」를 바로 못 답함 → 상태에서 다시 계획하는 질의 함수가 필요 |
| 관측 이력 · 시간 · 단계 입력 | 숨은 변수(궤적 시간 · 단계)를 관측으로 | 경과 시간 입력은 멈춤을 못 풂(같은 시간에 다른 단계), 직전 행동은 copycat |
| 입력 노이즈 증강 | 경계 근처 매끄럽게 | 직전 행동 입력에 노이즈 0.5 → copycat은 줄지만 다시 멈춤 |

- **랜덤화로 드러난 것(함태훈 week09)**: 청킹 25 BC는 랜덤화 없음 36 % → 근거 범위 동시 랜덤화 **8 %**(전문가 92 %). BC 민감 1 ~ 3위 = **명령 지연 20 ms(0/25)** · 테이블 −10 mm(0/25) · 팔 질량 −10 %(4/25) — 전문가는 기준 궤적 + 피드백이라 지연 40 ms도 견디지만, BC는 시연에 없던 동역학 · 관측(테이블 높이 1개)에서 무너짐 = 공변량 이동. 대책 = 랜덤화 환경에서 시연 재수집 + 관측에 직전 명령(지연 보상) · 손가락 접촉
- 구술용 한 문장: 「BC는 전문가 상태 분포에서만 배워서, 실행 중 작은 오차로 처음 보는 상태에 가면 오차가 T²로 쌓인다. 우리는 시연에 노이즈를 넣어 복귀 예시를 만들고(DART), 여러 주기를 한 번에 예측해(청킹) 결정 횟수와 멈춤 모호성을 줄였다. 근본 해결은 정책이 간 상태를 전문가가 다시 라벨링하는 DAgger다.」

## Q2. 우리 MuJoCo 시나리오의 각 요소가 Isaac Lab Manager-Based의 어떤 term에 대응하는가

| MuJoCo (우리 코드) | Manager | Isaac Lab term (예) |
|---|---|---|
| `pick_env.build()` 로봇 + 손가락 + 테이블 + 물체 | Scene | `InteractiveSceneCfg`: `B1_PICK_CFG`(ArticulationCfg, 손가락 URDF 사본) · `RigidObjectCfg` ×3 · `ContactSensorCfg` ×2 · `FrameTransformerCfg`(ee) |
| `sample_task(seed)` + `reset()` (물체 종류 · 위치 · yaw · 질량) | **Event (reset)** | `reset_scene_to_default`, `reset_root_state_uniform`, `randomize_rigid_body_mass` |
| 랜덤화(마찰 · 파지력 · armature · damping · kp · 베이스 질량 · kv) | **Event (startup/reset)** | `randomize_rigid_body_material`, `randomize_actuator_gains`, `randomize_joint_parameters`, `randomize_rigid_body_mass` — 함태훈 EventTerm 매핑과 통합 |
| `bc_common.features()` 39차원 | **Observation** (group policy) | `joint_pos` · `joint_vel`(왼팔), 나머지는 로봇 좌표 변환이 필요해 사용자 정의 함수(루트 = odom 고정이라 `root_*` 기본 함수 못 씀) |
| `apply_action()` action 8 | **Action** | 베이스 = 사용자 정의 `BaseBodyVelocityAction`(로봇 좌표 → R(yaw) → 관절 속도), 팔 = `RelativeJointPositionAction`(BC 관절) / `DifferentialInverseKinematicsAction`(BC 끝점) / `JointPositionAction`(재생), 그리퍼 = 1값 → 2관절 |
| `expert._judge()` 성공 · `eval_bc` 실패 분류 | **Termination** | 성공 = 사용자 정의 `success_placed`(0.5 s 유지), `time_out`(34 s), `illegal_contact`(로봇 ↔ 테이블, 손가락 제외), `root_height_below_minimum`(물체 낙하), NaN 가드 |
| (없음 — 전문가 · BC는 보상 안 씀) | **Reward** | RL 단계용 초안: 접근 · 파지 · 들기 · 목표 접근 · 놓기 성공 · 행동 변화율 벌점 |
| 목표 원(고정) | Command | 지금은 상수, 무작위화하면 `CommandTerm` |
| `expert.Expert` 상태기계 | (환경 밖) | 스크립트 정책 — 환경은 관측 · 행동만, 시연 저장은 `RecorderManager` |
| dt 0.5 ms × 40 = 50 Hz | Sim | `sim.dt` · `decimation`(PhysX 시작값 2 ms × 10) |

- 핵심 차이 3개: ① 로봇 좌표 기준이 루트가 아니라 베이스 yaw 관절 → 관측 · 행동 둘 다 사용자 정의 ② MuJoCo 접촉(solref · solimp · condim · elliptic)에 대응하는 PhysX 항이 없음 → 미끄러짐 표 재현으로 dt · 재질을 다시 정함 ③ 손가락 equality → 같은 목표 2관절(mimic 대신)

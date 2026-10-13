# 골격 · v0 URDF — 구동기 사양표 + 가정값 표 (week04)

| 항목 | 내용 |
|---|---|
| 상태 | **v0** (week04, 박종진, 2026-10-13) — 팀장 지시(10/8): BOM 구동기 사양 수령 전까지 **가정값 + 가정값 표(값 · 출처 · 확정 시 교체 항목)** 로 제출 |
| 모델 | [v0/](v0/) — `urdf/b1_humanoid_skeleton.urdf`(형상 없음) · `urdf/b1_humanoid_v0.urdf`(메시 · CAD 관성) · `mjcf/` 3단(robot → robot_actuated → scene) |
| 값 파일 | [assumptions_v0.json](../../members/park-jongjin/week04/scripts/assumptions_v0.json) — URDF · MJCF는 이 파일에서 자동 생성 ([build_v0_urdf.py](../../members/park-jongjin/week04/scripts/build_v0_urdf.py)). **값을 바꿀 땐 json만 고치고 다시 생성** |
| 근거 | 링크 · 관절 위치 = [link-split-v0.md](link-split-v0.md) §3 (STEP), 범위 · 토크 검증 = [v0_analysis.json](../../members/park-jongjin/week04/results/v0_analysis.json), 점검 = [checklist_v0.md](../../members/park-jongjin/week04/results/checklist_v0.md) (25항목 OK) |

## 1. 구동기 사양표 (joint ↔ STEP 구동기 ↔ URDF/MJCF)

| joint (left / right) | STEP 구동기 (left / right) | effort [N·m] | velocity [rad/s] | armature [kg·m²] | damping / friction | lower / upper [rad] | MJCF 액추에이터 |
|---|---|---|---|---|---|---|---|
| `{side}_shoulder_pitch_joint` | RI85:1 / RI85:6 | 100 (가정) | 3.0 (가정) | 0 (E6) | 0.5 / 0 (가정) | −3.0 / 3.0 | position kp 300, dampratio 1 |
| `{side}_shoulder_roll_joint` | RI85:2 / RI85:5 | 100 | 3.0 | 0 | 0.5 / 0 | **−0.28** / 1.5 | 〃 |
| `{side}_shoulder_yaw_joint` | RI85:3 / RI85:7 | 100 | 3.0 | 0 | 0.5 / 0 | −2.5 / 2.5 | 〃 |
| `{side}_elbow_joint` | RI85:4 / RI85:8 | 100 | 3.0 | 0 | 0.5 / 0 | **−1.83** / 1.83 | 〃 |
| `base_planar_joint` (TX · TY · RZ) | 하체 옴니휠 (실측 보류) | – | 1.5 m/s · 1.57 rad/s | – | – | 무제한 | velocity kv 200 / 50 |

- **effort = 피크 토크**를 넣는다 (시뮬레이터 힘 제한 = 물리적으로 낼 수 있는 최대값). 정격은 제어기 · 안전 리밋(B2 · B4) 쪽 — [actuator-spec-request.md §3](../docs/icd/actuator-spec-request.md)
- URDF effort → MuJoCo 관절 `actuatorfrcrange` ±100 + 액추에이터 `forcerange` ±100 (E4 두 군데 제한). `ctrlrange`는 `inheritrange="1"`로 관절 범위 상속 (명령 각도 범위 ≠ 힘 범위)
- URDF velocity는 MuJoCo 로드 시 버려짐(A5) — 속도 제한은 제어기에서

## 2. 가정값 표 (값 · 출처 · 확정 시 교체)

| No. | 항목 | v0 값 | 출처 / 근거 | 확정 시 교체 (누가 · 무엇으로) |
|---|---|---|---|---|
| S1 | RI85 피크 토크 → effort | 100 N·m (전 관절 동일) | 기획서 목표 「어깨 100 N·m급」. 팔 8관절 모두 같은 RI85라 같은 값 (기획서 「팔꿈치 40 N·m급」은 요구치일 뿐 구동기 값 아님) | A2 · 하이퍼다인 RI85 사양 (ICD 요청서 항목 2) |
| S2 | RI85 최고속도 → velocity | 3.0 rad/s (≈ 29 rpm) | 근거 없음 — 협동로봇 관절 일반 범위에서 보수적으로 | 사양 항목 3 |
| S3 | armature | 0 | 체크리스트 E6 (BOM 전 0 허용) | J_rotor · N² (항목 1 · 4) |
| S4 | damping / friction | 0.5 N·m·s/rad / 0 | 안정화용 작은 값 | 항목 6, 실측(다이나모) |
| S5 | 관절 범위 | §3 | roll 하한 · elbow는 **충돌 스윕**, 나머지는 케이블 여유 가정 | A1 기구 스토퍼 · 케이블 경로 |
| S6 | 링크 질량 · 관성 | STEP 체적 × 재질 가정 (기어 · 축 · 베어링 = 강 7.85, 나머지 Al 2.70 g/cm³) | [step_to_links.py](../../members/park-jongjin/week04/scripts/step_to_links.py) — left 팔 14.69 kg, RI85 1개 ≈ 2.67 kg | **함태훈 질량 · 관성 비교표(FreeCAD 재질 지정 · BOM)** · A4 실측 |
| S7 | 좌/우 · 전방 | CAD +z = 로봇 전방 → CAD x+ 팔 = left | CAD만으로 판단 불가 (link-split F3) | 팀장 확인 (10/13 질의) |
| S8 | 영점 자세 | STEP 자세 = 모든 관절 0 (팔 아래로 곧게, 팔꿈치 축 = 전후축) | ICD 3.2 | 팀장 확인 |
| S9 | 이상화 | J3 · J4 축 1.4° 기울기 · right 팔 x 7 mm 어긋남 무시, **right = left 거울** (ICD 3.4) | link-split §4 D1 · D2 | A1 CAD 수정본 |
| S10 | 베이스 | 상판 576 × 472 × 12 mm(STEP) 위 하체 box, 상판 윗면 높이 **0.40 m**, 지상고 0.05 m, 하체 **30 kg** + 상판 3.69 kg | 하체 실측표 보류(함태훈 week02) · 기획서 총중량 ≤ 80 kg 에서 역산 | 하체 실측 (높이 · 질량 · COM) |
| S11 | 몸통 | 어깨 J1축 높이 **1.20 m**, box 0.12 × 0.15 m (폭 = 양 J1 RI85 뒷면 간격 150 mm), **8 kg** + J1 RI85 고정측 2개(4.74 kg) | 기획서 전고 1,400~1,600 mm · STEP 어깨 간격 | A1 몸통 설계 |
| S12 | 제어 게인 | kp 300 N·m/rad, dampratio 1 / 베이스 kv 200 · 50 | week03 계단응답 (kp↑ → 처짐↓, 포화 주의) | B2 튜닝 |
| S13 | 손목 · 그리퍼 | 없음 (CAD 4-DOF) — tool0 = 그리퍼 플랜지면 | link-split F1 | 손목 2축 추가 시 링크 2개 · 관절 2개 추가 |

## 3. 관절 범위 근거 — 충돌 스윕 (convex hull collision, 다른 관절 0, 1° 간격)

| joint | + 방향 첫 접촉 | − 방향 첫 접촉 | v0 범위 |
|---|---|---|---|
| shoulder_pitch | 없음 (±180°) | 없음 | ±3.0 rad (케이블 가정) |
| shoulder_roll | 없음 | **19°** (elbow_link ↔ torso box, 내전) | −0.28 rad(16°) ~ 1.5 rad |
| shoulder_yaw | 없음 | 없음 | ±2.5 rad (케이블 가정) |
| elbow | **108°** (elbow ↔ shoulder_roll) | **108°** | ±1.83 rad(105°) |

- convex hull은 실제보다 뚱뚱 → **보수적**(실제는 더 넓을 수 있음). roll 내전 한계는 **가정한 몸통 폭 150 mm**에 좌우됨 → 몸통 설계 후 재스윕
- 범위 양 끝 16자세 접촉 0 (checklist D3)

## 4. 토크 여유 검증 (PDR 기준 「액추에이터 토크 여유 20% 이상」 사전 확인)

| 자세 (left 팔) | 페이로드 | 필요 정적 토크 | 100 N·m 가정 대비 여유 |
|---|---|---|---|
| shoulder_pitch −90° (팔 앞으로 수평, 최악) | 0 kg | **23.8 N·m** (손계산 = MuJoCo, F3) | +320 % |
| 〃 | **3 kg** (기획서 페이로드) | **38.4 N·m** | +161 % |
| shoulder_roll 1.5 rad(86°) 외전 | 3 kg | 35.2 N·m | +184 % |
| elbow 90° (전완 수평) | 3 kg | 7.6 N·m | – |

→ **RI85 피크가 46 N·m 이상(38.4 × 1.2)이면 PDR 20 % 여유 충족.** 정적 중력만 본 값 — 가속 · 동적 토크는 B3 궤적 확정 후. 리치 495 mm(목표 700 mm급)로 늘어나면 토크는 거의 비례 증가

## 5. 자기 점검 (체크리스트 v0.3) — [checklist_v0.md](../../members/park-jongjin/week04/results/checklist_v0.md)

25항목 OK (A1~A8 · B1~B6 · C3 · D1 · D3 · D4 · E1 · E4~E7 · F1~F3 · F5). 주요 수치: nq = nv = 11 (planar 3 + 팔 8), nu 11, nbody 14, 총질량 75.80 kg, 영점 접촉 0, 수평 처짐 0.0791 rad = τg/kp 0.0794 (E7)

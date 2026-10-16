# B1 공용 가정값 표 v0.1 — 구동기 사양표 + 가정값 표

| 항목 | 내용 |
|---|---|
| 상태 | **v0.2 — B1 공용** (week05, 박종진, 2026-10-16) · v0.1 = 팀장 확정 10/16, **v0.2 = S14 그리퍼 임시 box 추가(5주차 커리큘럼, 팀장 확인 필요)**. **박종진 · 함태훈 URDF/MJCF 모두 이 표의 값을 쓴다** (v0 때 두 모델 총질량 74.10 vs 67.46 kg → 통일) |
| 이전 | v0 (week04, 2026-10-13, 박종진 개인 가정값 표) — 태그 week04 (d86aa1c) |
| 값 파일 | **[assumptions_b1.json](assumptions_b1.json)** (B1 공용, v0.2 — v1 빌드 [v1/scripts/build_v1.py](v1/scripts/build_v1.py)가 읽음). v0 생성에 쓴 [week04 assumptions_v0.json](../../members/park-jongjin/week04/scripts/assumptions_v0.json)과 수치 동일(주석 · 밀도 · 팔 항목만 추가) → **v0 모델은 재생성 불필요**. v1부터 공용 json 하나만 읽어 생성 |
| 모델 | **[v1/](v1/)** (week05, 양팔 + 베이스 3-DOF + 상판 링크 + 그리퍼, 검증 5종 [validate_model.md](v1/results/validate_model.md)) · [v0/](v0/) — `urdf/b1_humanoid_skeleton.urdf` · `urdf/b1_humanoid_v0.urdf` · `mjcf/` 3단(robot → robot_actuated → scene) |
| 근거 | 링크 · 관절 위치 = [link-split-v0.md](link-split-v0.md) §3, 범위 · 토크 = [v0_analysis.json](../../members/park-jongjin/week04/results/v0_analysis.json), 점검 = [checklist_v0.md](../../members/park-jongjin/week04/results/checklist_v0.md), 두 모델 정합 = [b1_vs_taehoon_mass_torque.md](../../members/park-jongjin/week05/results/b1_vs_taehoon_mass_torque.md) |

## 0. B1 공용 확정값 (팀장 10/16) — 한눈에

| No. | 항목 | 공용 값 | 비고 |
|---|---|---|---|
| C1 | 베이스 | 하체 **30 kg** + 상판 **3.69 kg (별도 링크)**, 상판 윗면 **0.40 m** | 상판 = `base_plate_link`(fixed)로 v1 재생성부터 분리. v0는 base_link 안 합성(질량 · COM 동일) |
| C2 | 몸통 | box **8 kg** + J1 RI85 고정측(CAD), 어깨 J1축 높이 **1.20 m** | 함태훈 v0: box 5 kg · 어깨 0.80 m → 맞춤 |
| C3 | 관절 범위 | **충돌 스윕 값**(§3): pitch ±3.0 · roll −0.28~1.5 · yaw ±2.5 · elbow ±1.83 rad | **roll 하한 −0.28은 몸통 폭 150 mm 가정(S11)에 종속** — 몸통 설계 확정 시 재스윕 |
| C4 | 구동기 | effort **100** N·m · velocity **3.0** rad/s · armature **0** · damping **0.5** | effort = 피크. 함태훈 v0: effort 50 · damping 없음 → 맞춤 |
| C5 | 제어 | position **kp 300**, **dampratio 1** | dampratio는 MuJoCo가 컴파일 시 영점 관성으로 kv 계산. **kv를 고정값으로 옮겨 쓰면 v1에서 관성이 바뀔 때 재튜닝** |
| C6 | 밀도 | **Al 2700 / 강 7850 / PCB 1850 kg/m³** (함태훈 3종) | sun adapter · planet carrier = Al (10/14 합의) |
| C7 | 팔 | 팔당 **4-DOF 확정**(손목 없음, A1 확인 10/16), 오른팔 = 왼팔 **거울(안 A, 승인 10/16)** | 기획서 6-DOF와 불일치 → 팀장이 상위 보고 |
| C8 | 베이스 관절 | **C3 평면 3-DOF 채택**: URDF `planar` 1개 유지(MuJoCo `_TX · _TY · _RZ` 자동 분해) | Isaac Sim 이식 시 slide x · slide y · revolute z **3관절로 분해** (체크리스트 C3-1 비고) |

**정합 확인 (10/16)** — 함태훈 v0 URDF(f9c9eff)에 C1 · C2 · C7을 적용하면 총질량 **74.093 kg = B1 74.096 kg**(차 0.003 kg = B1 더미 프레임 3 × 0.001 kg), 팔 수평 중력토크 **22.73 / 37.30 N·m (0 / 3 kg) 일치(차 0.005 N·m)**. 차이 분해 원본 74.10 − 67.46 = 6.63 kg = 상판 +3.69 · 몸통 box +3.00 · 오른팔 −0.06 (B1 거울 vs 함태훈 CAD 오른팔) — [b1_vs_taehoon_mass_torque.md](../../members/park-jongjin/week05/results/b1_vs_taehoon_mass_torque.md), 재현 `python members/park-jongjin/week05/scripts/compare_b1_taehoon.py b1/models/v0/urdf/b1_humanoid_v0.urdf <함태훈 our_robot_v0.urdf> out.md`

## 1. 구동기 사양표 (joint ↔ STEP 구동기 ↔ URDF/MJCF)

| joint (left / right) | STEP 구동기 (left / right) | effort [N·m] | velocity [rad/s] | armature [kg·m²] | damping / friction | lower / upper [rad] | MJCF 액추에이터 |
|---|---|---|---|---|---|---|---|
| `{side}_shoulder_pitch_joint` | RI85:1 / RI85:6 | 100 (가정) | 3.0 (가정) | 0 (E6) | 0.5 / 0 (가정) | −3.0 / 3.0 | position kp 300, dampratio 1 |
| `{side}_shoulder_roll_joint` | RI85:2 / RI85:5 | 100 | 3.0 | 0 | 0.5 / 0 | **−0.28** / 1.5 | 〃 |
| `{side}_shoulder_yaw_joint` | RI85:3 / RI85:7 | 100 | 3.0 | 0 | 0.5 / 0 | −2.5 / 2.5 | 〃 |
| `{side}_elbow_joint` | RI85:4 / RI85:8 | 100 | 3.0 | 0 | 0.5 / 0 | **−1.83** / 1.83 | 〃 |
| `base_planar_joint` (TX · TY · RZ) | 하체 옴니휠 (실측 보류) | – | 1.5 m/s · 1.57 rad/s | – | – | 무제한 | velocity kv 200 / 50 |

- **effort = 피크 토크** (시뮬레이터 힘 제한 = 물리적으로 낼 수 있는 최대값) — [actuator-spec-request.md §3](../docs/icd/actuator-spec-request.md). **정격은 제어기 · 안전 리밋(B2 · B4)과 PDR 토크 여유 판정(§4)에 쓴다**
- URDF effort → MuJoCo 관절 `actuatorfrcrange` ±100 + 액추에이터 `forcerange` ±100 (E4 두 군데 제한). `ctrlrange`는 `inheritrange="1"`로 관절 범위 상속 (명령 각도 범위 ≠ 힘 범위)
- URDF velocity는 MuJoCo 로드 시 버려짐(A5) — 속도 제한은 제어기에서

## 2. 가정값 표 (값 · 출처 · 확정 시 교체)

| No. | 항목 | v0.1 공용 값 | 출처 / 근거 | 확정 시 교체 (누가 · 무엇으로) |
|---|---|---|---|---|
| S1 | RI85 피크 토크 → effort | 100 N·m (전 관절 동일) | 기획서 목표 「어깨 100 N·m급」. 팔 8관절 모두 같은 RI85 (기획서 「팔꿈치 40 N·m급」은 요구치) | A2 · RI85 사양 (ICD 요청서 항목 2) — **회신 대기** |
| S2 | RI85 최고속도 → velocity | 3.0 rad/s (≈ 29 rpm) | 근거 없음 — 협동로봇 관절 일반 범위에서 보수적으로 | 사양 항목 3 |
| S3 | armature | 0 | 체크리스트 E6 (BOM 전 0 허용) | 출력축 기준 반사관성 (항목 4-1) |
| S4 | damping / friction | 0.5 N·m·s/rad / 0 | 안정화용 작은 값 | 항목 6, 실측(다이나모) |
| S5 | 관절 범위 | §3 (C3) | roll 하한 · elbow는 **충돌 스윕**, 나머지는 케이블 여유 가정. **roll 하한은 몸통 폭 150 mm 가정에 종속** | A1 기구 스토퍼 · 케이블 경로 · 몸통 설계 후 재스윕 |
| S6 | 링크 질량 · 관성 | 함태훈 메시 · inertial(f9c9eff) — 왼팔 14.17 kg, 오른팔 = 왼팔 거울, 총 **74.10 kg** | 밀도 C6 (Al 2700 / 강 7850 / PCB 1850, FreeCAD 재질 지정). B1 임시값(강 기어 가정, 팔 14.81 kg)은 대조용 — [step_to_links.py](../../members/park-jongjin/week04/scripts/step_to_links.py) | 함태훈 v1 관성표(골격 관절점 원점) · BOM · A4 실측 |
| S7 | 좌/우 · 전방 | base_link = **REP 103**. **CAD → base 고정 변환 하나**(URDF `cad_frame`, json `cad_to_base`: x_r = z_c, y_r = x_c + 118.15, z_r = y_c), 좌/우 = 변환 후 **+y 부호** | +z_CAD = 전방은 **가정 유지** | A1 정면 확정(**회신 대기**) → `cad_to_base.R`만 교체 · 재생성 |
| S8 | 영점 자세 | **팔 내린 자세 + 팔꿈치 전방 굽힘** = 모든 관절 0. `shoulder_yaw` origin rpy (0, 0, ∓π/2) | **팀장 확정 10/14** | – |
| S9 | 공칭 · 편차 | URDF = **공칭값(축 직교 · 좌우 대칭, right = left 거울)**. CAD 편차(J3 · J4 1.37°, right 7.2~7.5 mm)는 [joint-origin-check-v0.md](joint-origin-check-v0.md) | 팀장 확정 10/14 | A1 회신(**대기**) — 의도된 설계면 공칭값은 두고 origin rpy에 명시적 오프셋 |
| S10 | 베이스 | 상판 576 × 472 × 12 mm(STEP) 위 하체 box, 상판 윗면 **0.40 m**, 지상고 0.05 m, 하체 **30 kg** + 상판 **3.69 kg(별도 링크, v1부터)**. 표현 = **C3 평면 3-DOF (확정 10/16)** | 하체 실측표 보류 · 기획서 총중량 ≤ 80 kg 에서 역산 | 하체 실측 (높이 · 질량 · COM) |
| S11 | 몸통 | 어깨 J1축 높이 **1.20 m**, box 0.12 × 0.15 m (폭 = 양 J1 RI85 뒷면 간격 150 mm), **8 kg** + J1 RI85 고정측 2개(CAD 4.06 kg) | 기획서 전고 1,400~1,600 mm · STEP 어깨 간격 | A1 몸통 설계 (폭 바뀌면 S5 재스윕) |
| S12 | 제어 게인 | kp 300 N·m/rad, **dampratio 1** / 베이스 kv 200 · 50 | week03 계단응답. dampratio → kv는 컴파일 시 계산, **고정 kv로 쓰면 v1 관성 변경 시 재튜닝** | B2 튜닝 |
| S13 | 손목 · 그리퍼 | **없음 — 4-DOF 확정** (A1 확인 10/16: 손목 없음, 4-DOF가 설계). 자리표시자 링크 · 관절 두지 않음, tool0 = 그리퍼 플랜지면 | link-split F1 · 팀장 10/16 | 기획서 6-DOF 불일치는 팀장 상위 보고 — 최종 사양인지 이번 제작분만인지 확인 중 |
| S14 | **그리퍼 임시 형상** (v0.2) | **0.5 kg box 80 × 80 × 100 mm**, tool0(플랜지면)에서 전완 연장 방향, 끝단 `{side}_ee` site(+0.10 m) | 5주차 커리큘럼 「그리퍼 = BOM 질량 box」 — **BOM 미수령이라 질량 · 치수 근거 없음**. 팔 수평 토크 22.73 → 25.40 N·m(0 kg), 3 kg 페이로드를 ee에 두면 37.30 → **42.91 N·m** | A2 · BOM 그리퍼 질량 · 치수 · TCP 위치 |

## 3. 관절 범위 근거 — 충돌 스윕 (convex hull collision, 다른 관절 0, 1° 간격, v0 통합 모델)

| joint | + 방향 첫 접촉 | − 방향 첫 접촉 | 공용 범위 |
|---|---|---|---|
| shoulder_pitch | 없음 (±180°) | 없음 | ±3.0 rad (케이블 가정) |
| shoulder_roll | 없음 | **22°** (elbow_link ↔ torso, 내전) | −0.28 rad(16°) ~ 1.5 rad |
| shoulder_yaw | 없음 | 없음 | ±2.5 rad (케이블 가정) |
| elbow | **109°** (elbow ↔ shoulder_roll) | **108°** | ±1.83 rad(105°) |

- 수치 = v0 통합 모델(함태훈 메시) 재스윕 결과([v0_analysis.json](../../members/park-jongjin/week04/results/v0_analysis.json)). v0 문서의 roll 19° · elbow 108°는 B1 임시 메시 값
- convex hull은 실제보다 뚱뚱 → **보수적**. roll 내전 한계는 **가정한 몸통 폭 150 mm**에 좌우됨 → 몸통 설계 후 재스윕
- 범위 양 끝 16자세 접촉 0 (checklist D3)

## 4. 토크 여유 검증 (PDR 기준 「액추에이터 토크 여유 20% 이상」 사전 확인) — v0 통합 모델

| 자세 (left 팔) | 페이로드 | 필요 정적 토크 | 정격 하한 (× 1.2) |
|---|---|---|---|
| shoulder_pitch −90° (팔 앞으로 수평, 최악) | 0 kg | **22.73 N·m** (손계산 = MuJoCo, checklist F3) | 27.3 N·m |
| 〃 | **3 kg** (기획서 페이로드) | **37.30 N·m** | 44.8 N·m |
| 〃 — 강 상한 (sun adapter · carrier 강 가정, B1 임시) | 3 kg | 38.4 N·m | **46 N·m** |
| shoulder_roll 1.5 rad(86°) 외전 | 3 kg | 34.25 N·m | 41.1 N·m |
| elbow +90° (전완 앞 수평) | 3 kg | 7.62 N·m (elbow) | 9.1 N·m |

> **v1 반영 결과 (S14 그리퍼 0.5 kg + 페이로드 작용점 플랜지 → ee 0.10 m)** — 검증 ③ [v1/results/validate_model.md](v1/results/validate_model.md): shoulder_pitch 3 kg **42.91 N·m → × 1.2 = 51.5 N·m > 46** (정격 46 가정 시 여유 7.2 %), shoulder_roll 38.89 N·m (여유 18.3 %). **그리퍼 질량 · TCP 위치가 정해지면 판정 기준을 다시 잡아야 함** — 팀장 보고 사항

→ **판정 기준(팀장 10/16): RI85 정격 토크 ≥ 46 N·m이면 PDR 20 % 여유 충족** (강 상한 38.4 × 1.2, 보수적). 피크는 이 기준과 별개로 **가속 · 동적 토크분**을 위한 여유로 B3 궤적 확정 후 따로 본다. effort(시뮬 힘 제한)는 계속 피크(요청서 §3).
- v0 문서의 23.8 / 35.2 N·m는 B1 임시 관성 값 → 통합 모델 값으로 교체(checklist F3와 일치). 리치 495 mm(목표 700 mm급)로 늘어나면 토크는 거의 비례 증가

## 5. 자기 점검 (체크리스트 v0.3) — [checklist_v0.md](../../members/park-jongjin/week04/results/checklist_v0.md)

25항목 OK. 주요 수치(v0 통합): nq = nv = 11 (planar 3 + 팔 8), nu 11, nbody 15, 총질량 **74.10 kg**, 영점 접촉 0, 수평 처짐 시뮬 0.0755 rad vs τg/kp 0.0758 rad (E7, kp 300)

## 6. 관절 축 방향(+/−) 규약 — **안 A 거울 확정** (B1 합의 10/14, **팀장 승인 10/16**)

- 축은 각 링크 프레임 기준. 영점 재정의(S8) 후 elbow 축은 base에서 −y (양팔 모두 + = 전방 굽힘)
- 실제 구동기 출력 +방향과 다른 관절 = [joint-origin-check-v0.md](joint-origin-check-v0.md) §2 B2 부호표 — **부호 반전은 드라이버 계층 한 곳에서만** (URDF · 제어기 · ICD는 이 표의 +방향 그대로)

| | shoulder_pitch | shoulder_roll | shoulder_yaw | elbow |
|---|---|---|---|---|
| left | +y | +x | +z | +x |
| right (안 A 거울, 확정) | +y | −x | −z | −x |

- 같은 명령 → 좌우 대칭 동작, 관절 범위 양팔 동일, ICD v0 3.4(SRR 제출)와 일치. (검토했던 안 B = 오른팔도 +y · +x · +z · +x, 범위 좌우 반대 — 채택 안 함)

## 7. 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| v0 | 2026-10-13 | 박종진 개인 가정값 표 S1~S13 (week04) |
| v0.1 | 2026-10-16 | **B1 공용 승격**(§0 C1~C8, 공용 json `assumptions_b1.json`) · 손목 없음 4-DOF 확정(S13) · C3 평면 3-DOF 확정(S10) · 안 A 거울 승인(§6) · PDR 토크 여유 = **정격 ≥ 46 N·m**(§4, 피크는 가속분 별도) · §3 · §4 · §5 수치를 v0 통합 모델 값으로 갱신(23.8 → 22.73 등) · 밀도 3종(C6) · kv 재튜닝 비고(C5) · 함태훈 URDF 정합 확인 |
| **v0.2** | 2026-10-16 | S14 그리퍼 임시 box(0.5 kg, 5주차 커리큘럼) · 상판 별도 링크 v1 적용 · §4 v1 반영 결과(3 kg 42.91 N·m → 정격 기준 51.5 N·m) |

# ICD 반영 목록 v0 — ICD 요청 초안 · G1 변경 목록 → v0 URDF / MJCF

| 항목 | 내용 |
|---|---|
| 상태 | **v0** (week04, 박종진, 2026-10-14) — 커리큘럼 4주차 개별 산출물 「ICD 반영 목록」 |
| 대상 모델 | [v0/urdf/b1_humanoid_v0.urdf](v0/urdf/b1_humanoid_v0.urdf) · [v0/mjcf/](v0/mjcf/) (함태훈 메시 · inertial + 박종진 골격, 74.10 kg) |
| 기준 문서 | ICD 요청 초안 [icd-v0-request-draft.md](../docs/icd/icd-v0-request-draft.md) (week02) · G1 변경 목록 [g1-structure-tree.md](../../members/park-jongjin/week02/g1-structure-tree.md) (week02) |
| 판정 | **반영** = v0에 그대로 적용 · **변경** = 팀장 결정 · 실제 CAD 때문에 바꿔서 적용 · **보류** = 입력 미수령 |

## 1. ICD 요청 초안 → v0

| ICD | 규약 | 판정 | v0에서 한 것 | 근거 |
|---|---|---|---|---|
| 1.1 | STEP(AP214) + 링크 단위 메시 | 반영 | STEP 1부(양팔 · 상판) 수령, 링크 메시는 함태훈이 STEP에서 생성(18개) | link-split-v0 · meshes/ |
| 1.2 | 링크 메시는 링크 좌표계 원점 기준 | **변경** | 함태훈 메시 원점이 골격 관절점과 축 방향 17~22 mm 차이 → `merge_taehoon.py`에서 평행이동 보정. **v1부터 함태훈이 골격 관절점(출력 디스크 중심) 기준으로 내보냄**(10/14 합의) → 보정 0, 0.5 mm 이내 확인만 | week04/log §4 |
| 1.3 | 단위 m | 반영 | STL mm → URDF `scale 0.001`, 질량 kg · 관성 kg·m² | checklist_v0 |
| 1.4 | visual / collision 분리 | 반영 | visual = 메시, collision = convex hull(MuJoCo 자동) | assumptions §3 |
| 1.5 | 링크 = 한 강체 묶음 | 반영 | 부품 접촉(거리 0)으로 분할, **RI85는 고정측(부모) / 출력측 output-v1 + ring2 + cover plate1(자식)** 으로 나눔 | link-split-v0 §2 |
| 2 | 질량 · 관성: 링크 좌표계 COM, COM 기준 텐서 | 반영 | 함태훈 표(f9c9eff)를 링크 프레임으로 옮겨 inertial 블록, balanceinertia off | link_inertials_taehoon_in_b1frame.json |
| 3.1 | REP-103 (x 전방 · y 좌 · z 상) | 반영 | CAD 축을 그대로 쓰지 않고 **고정 변환 1개**(`cad_frame`), 좌/우 = 변환 후 +y 부호. +z_CAD = 전방은 가정(팀장 10/14) | assumptions S7 |
| 3.2 | 영점 = 팔 아래로 곧게 | **변경** | 팀장 10/14 재정의: **팔 내린 자세 + 팔꿈치 전방 굽힘**. CAD 자세와의 yaw 90° 차이는 `shoulder_yaw` origin rpy (0, 0, ∓π/2)로 흡수 | assumptions S8 |
| 3.3 | 링크 원점은 관절축 위, 축은 단위축 정렬 | 반영 | 원점 = RI85 출력 디스크 중심, 축 = 공칭(직교). CAD 편차(J3 · J4 1.37°, right 7.2~7.5 mm)는 검증표에 기록 → A1 확인 중 | joint-origin-check-v0 · S9 |
| 3.4 | +방향 오른손 법칙, 좌우 거울 | 반영(확인 대기) | 안 A 거울: left pitch +y · roll +x · yaw +z · elbow +x, right는 roll · yaw · elbow 부호 반전. 함태훈 동의(10/14), **팀장 확인 대기**. 구동기 출력 방향과 다른 관절은 B2 부호표로 넘김 | assumptions §6 · 검증표 §2 |
| 3.5 | 부모 → 자식 xyz · rpy · 축 · 범위 | 반영 | CAD 관절점에서 origin = R_pᵀ(p_c − p_p), 범위는 충돌 스윕 + 케이블 여유 가정 | link-split §3 · assumptions §3 |
| 4 | 이름 규칙 `{side}_{부위}_{운동}_link/joint` | 반영 | base_link · torso_link · left/right_shoulder_pitch/roll/yaw · elbow · tool0 (체크리스트 이름 항목 OK) | checklist_v0 |
| 4 | 센서 프레임(imu · camera) | **보류** | 센서 위치 미수령 → v0에 site 없음 | A팀 센서 배치 |

## 2. G1 변경 목록(week02) → v0

| G1 → 우리 로봇 | 판정 | v0 |
|---|---|---|
| 팔 7-DOF → 6-DOF | **변경** | CAD가 **팔 4-DOF**(RI85 × 4) → v0는 4-DOF(팀장 10/14), 손목 2축은 A1 계획 확인 중 (S13) |
| freejoint 다리 → 홀로노믹 베이스 | 반영 | `planar`(TX · TY · RZ) 관절 + velocity 액추에이터 3개 |
| 몸통 · 허리 | **변경** | 몸통 CAD 없음 → torso box 0.12 × 0.15 m, 8 kg, 허리 관절 없음 (S11) |
| position kp 500 → 우리 구동기 | 반영(임시) | position kp 300 · dampratio 1, forcerange ±100 N·m (S1 · S12). 토크(`motor`) 입력은 B2 제어 단계에서 |
| effort 어깨 100 · 팔꿈치 40 N·m급 | **변경** | 팔 전 관절 같은 RI85 → effort 100 N·m 가정(S1). 필요 37.3 N·m → 피크 ≥ 46 N·m면 20 % 여유 |
| integrator implicitfast | 반영 | scene.xml `implicitfast`, timestep 0.002 s (1 kHz 검토는 제어 단계) |
| 총질량 ≤ 80 kg | 반영 | 74.10 kg (하체 30 kg 가정 포함, S10) |

## 3. 남은 입력 (다음 버전에서 교체)

- BOM(재질 · 질량 · 구동기 모델) — 링크 분할표 BOM 열, S6 대조
- RI85 사양(피크 토크 · 속도 · 감속비 · 로터 관성) → S1~S4 · armature ([actuator-spec-request.md](../docs/icd/actuator-spec-request.md) v0.1)
- 전방 방향 · 축 부호 규약 팀장 확정, A1 편차 회신 · 손목 계획, 하체 실측, 센서 위치

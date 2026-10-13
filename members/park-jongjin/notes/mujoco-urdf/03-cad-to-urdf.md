# 03. CAD(STEP) → URDF — 4주차 구술 준비 (박종진)

근거 파일: [link-split-v0.md](../../../../b1/models/link-split-v0.md) · [assumptions-v0.md](../../../../b1/models/assumptions-v0.md) · [build_v0_urdf.py](../../week04/scripts/build_v0_urdf.py) · [checklist_v0.md](../../week04/results/checklist_v0.md)

## Q1. joint origin을 CAD 좌표에서 뽑을 때 부모 링크 프레임 기준으로 바꾸는 절차는?

1. **관절 축을 CAD에서 찾는다** — 축 = 구동기 출력 디스크(`output-v1`)의 관성 주축(대칭축), 위치 = 그 디스크 중심. 방향은 고정측 → 출력측 ([step_joint_axes.py](../../week04/scripts/step_joint_axes.py)). 예: left 어깨 pitch (51.6, 0, 0) mm, 축 +x_CAD
2. **좌표계를 ICD로 바꾼다** — CAD는 y 위 · x 좌우, ICD는 x 전방 · y 좌측 · z 위 → `x_r = z_c, y_r = x_c − (−118.15), z_r = y_c`, mm → m (×0.001). 회전행렬 R = [[0,0,1],[1,0,0],[0,1,0]], 축벡터도 R로 회전
3. **각 링크 프레임 원점 = 그 링크를 움직이는 관절 점**으로 둔다 (영점 자세에서 모든 링크 프레임을 base와 같은 방향 → rpy = 0)
4. **joint origin xyz = 자식 관절 점 − 부모 관절 점** (부모 프레임 기준, rpy 0이라 회전 없이 차이만). 예: J2 − J1 = (107.9, −20, 0) mm_CAD → (0, 0.1079, −0.020) m
   - 일반식: 부모 프레임의 월드 자세가 (R_p, p_p)이면 `xyz = R_pᵀ (p_child − p_p)`, `rpy = R_pᵀ R_child`. 영점에서 R_p = I로 두면 단순 차이가 됨
5. **메시 · 관성도 같은 프레임으로** — 정점 `v_link = R (v_CAD − P_joint)/1000`, COM 같은 식, 관성은 `R I Rᵀ` (COM 기준 유지)
6. **검증** — MuJoCo로 열어 tool0 위치가 CAD 플랜지면과 같은지: left tool0 영점 (−0.009, 0.278, 0.705) m, pitch −90°에서 (0.495, 0.278, 1.191) = 어깨 높이 1.20 m − 9 mm, 리치 0.495 m ✓
- **한계**: CAD 조립 오차(J3 · J4 축 1.4° 기울어짐, right 팔 7 mm)는 이상화했다 → 실기와 tool0 위치가 최대 ~7 mm · 1.4° 다를 수 있음. 실기 캘리브레이션(B2)에서 보정

## Q2. BOM 정격 vs 피크 토크 중 effort에 무엇을? forcerange · ctrlrange와의 대응은?

| 값 | 넣는 곳 | 이유 |
|---|---|---|
| **피크** | URDF `effort` → MuJoCo joint `actuatorfrcrange` + 액추에이터 `forcerange` | 시뮬레이터 힘 제한은 "물리적으로 낼 수 있는 최대". 정격으로 자르면 실기가 짧게 낼 수 있는 토크를 시뮬이 못 냄 |
| 정격 | 제어기 · 안전 리밋, 학습 보상 토크 패널티 | 연속 운전 발열 한계 — 시간 개념이 있어 시뮬 힘 제한으로는 표현 불가 |
| 관절 range | `ctrlrange` (position은 `inheritrange="1"`) | ctrlrange = **명령(목표 각도)** 범위 [rad], forcerange = **출력 힘** 범위 [N·m] — 단위 · 의미가 다름 |

- 수치: v0는 effort 100 N·m(가정). week03 kp 100 실험에서 40 N·m 포화 → 오버슈트 이론 64.3 % vs 시뮬 60.7 % — 포화가 응답을 바꿈
- 필요 토크: 팔 앞으로 수평 + 3 kg → **38.4 N·m** (손계산 = MuJoCo). PDR 「토크 여유 20 %」 → **RI85 피크 ≥ 46 N·m** 필요
- 처짐: kp 300에서 0.0791 rad (이론 τg/kp = 0.0794) → 중력보상(B3) 없으면 4.5° 처짐

## 척도 5용 — 대안 · 한계 · 다음 단계 의견

1. **팔 4-DOF vs 목표 6-DOF**: CAD에 RI85 4개 → 자세(손끝 방향) 제어 불가, 위치 3 + 1 여유. 파지 데모라면 손목 2축(pitch · roll)이 필요 — 손목은 저토크(7.6 N·m 수준 이하)라 RI85보다 작은 구동기로 충분할 것 (팔꿈치 필요 토크 근거)
2. **리치 495 mm vs 700 mm급**: 링크를 늘리면 어깨 토크가 거의 비례 증가 → 38.4 × 700/495 ≈ 54 N·m, 여전히 100 N·m 가정 안이지만 RI85 실제 피크 확인이 먼저
3. **충돌 hull 한계**: convex hull은 U자 요크 사이를 채움 → 팔꿈치 108°, 내전 19°는 보수적. 정밀하게는 링크를 부품별 hull 여러 개로 쪼개기(CoACD 등)
4. **관성 3자 비교**: elbow_link OCCT vs trimesh 질량 0.11 %, 관성 ≤ 0.21 % 일치 → 메시 계산은 믿을 만함. 남은 오차는 **재질 가정**(BOM 질량 대조 필요)이 지배적
5. **베이스 planar**: Isaac Sim URDF Importer는 planar 관절을 지원 안 할 수 있음 → 실습 PC에서 확인, 안 되면 prismatic 2 + revolute 1 더미 링크로 전환

# joint 원점 검증표 v0 — URDF 공칭값 vs CAD(STEP) 실측

| 항목 | 내용 |
|---|---|
| 상태 | **v0** (week04, 박종진, 2026-10-14) — 팀장 지시(10/14): URDF v0는 **공칭값(축 직교 · 좌우 대칭)**, CAD 편차는 이 표에 기록 |
| 실측 | 각 RI85 `output-v1` 디스크의 대칭축(관성 주축) · 중심 — OCCT, [joint_origin_check.py](../../members/park-jongjin/week04/scripts/joint_origin_check.py) |
| 공칭 | `b1_humanoid_v0.urdf`를 MuJoCo로 읽고 **CAD 자세**(shoulder_yaw를 영점 rpy만큼 되돌림)로 둔 joint 축 · 앵커. CAD 점은 URDF의 `cad_frame`(CAD → base_link 고정 변환 하나)로 옮겨 비교 |
| 교차 검증 | 함태훈 [joint_check_skeleton.csv](https://github.com/taehoonham0118-cell/-1-/blob/main/sim/robot/joint_check_skeleton.csv) (e292b37 골격 기준): 어깨 ≤ 0.05 mm · 0.01°, yaw · elbow 1.37° · 7.4 mm — **일치** |
| 원자료 | [joint_origin_check_v0.csv](../../members/park-jongjin/week04/results/joint_origin_check_v0.csv) (실측 · 공칭 좌표 · 축 벡터, base_link 기준 mm) |

## 1. 편차

| joint | STEP 구동기 | 축 각도차 [°] | 축선 거리 [mm] | 실측 +방향 vs URDF |
|---|---|---|---|---|
| `left_shoulder_pitch_joint` | RI85 제작건:1 | 0.00 | 0.0 | 같음 |
| `left_shoulder_roll_joint` | RI85 제작건:2 | 0.01 | 0.0 | 같음 |
| `left_shoulder_yaw_joint` | RI85 제작건:3 | 1.37 | 2.2 | 반대(부호 규약) |
| `left_elbow_joint` | RI85 제작건:4 | 1.37 | 0.9 | 같음 |
| `right_shoulder_pitch_joint` | RI85 제작건:6 | 0.00 | 0.0 | 반대(부호 규약) |
| `right_shoulder_roll_joint` | RI85 제작건:5 | 0.01 | 0.1 | 반대(부호 규약) |
| `right_shoulder_yaw_joint` | RI85 제작건:7 | 1.37 | 7.5 | 같음 |
| `right_elbow_joint` | RI85 제작건:8 | 1.37 | 7.4 | 반대(부호 규약) |

- **축 각도차** = 실측 축과 공칭 축 사이 각 (부호 무시), **축선 거리** = 공칭 앵커에서 실측 축선까지 수직 거리
- 어깨 pitch · roll: 공칭 = CAD (≤ 0.06 mm, 0.01°)
- shoulder_yaw · elbow: **1.37°** (양팔 같음, D1) — J2 축과 J3 축 사이각 88.63° (설계 90° 예상). right는 여기에 **7.2 ~ 7.5 mm** 위치 편차(D2)가 더해짐
- → A1 확인 요청 그림: [20261014_a1_deviation_request.png](../../members/park-jongjin/week04/img/20261014_a1_deviation_request.png) (팀장 경유 전달)

## 2. 구동기 +방향 vs URDF 축 (B2 부호표 초안)

위 표의 「+방향」 = STEP 상 구동기 출력 방향(고정측 → 출력측)이 URDF 관절 축과 같은지. **반대인 관절은 B2에서 `q_URDF = −q_모터`**(엔코더 · 토크 부호 반전) 처리 필요. URDF 축은 좌우 거울 규약(ICD 3.4, assumptions-v0 §6)으로 정해 두고, 실제 구동기 방향 차이는 이 표로 넘긴다 — 실기 조립 방향이 확정되면 갱신.

> **주석 (팀장 10/16, 축 부호 규약 안 A 거울 승인)**: 부호 반전은 **드라이버 계층 한 곳에서만** 한다. URDF · MJCF · 제어기 · ICD는 모두 URDF 축 +방향(assumptions-v0 §6)을 그대로 쓰고, 위 「반대」 관절(left yaw, right pitch · roll · elbow)의 `q_URDF = −q_모터` · `τ_URDF = −τ_모터` 변환은 B2 모터 드라이버 래퍼 안에만 둔다 (두 군데서 반전하면 부호가 다시 뒤집힘).

## 3. 영점 재정의 (팀장 10/14)

- 영점 = **팔 내린 자세 + 팔꿈치 전방 굽힘**. CAD 자세(팔꿈치 축이 전후 → 옆으로 굽음)와의 차이 90°를 `{side}_shoulder_yaw_joint` origin **rpy (0, 0, ∓π/2)** (left −, right + 거울)로 흡수
- 결과: 양팔 elbow 축 = base −y, **elbow + = 전방 굽힘** (elbow +0.8 rad → tool0 x +0.143 m), 영점 접촉 0
- CAD 자세 재현: shoulder_yaw q = +π/2(left) → left tool0 (−0.009, 0.2777, 0.705) m = CAD 플랜지 중심을 cad_frame으로 옮긴 값 (−0.009, 0.2776, 0.705) m ✓

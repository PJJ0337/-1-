# week05 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 05 주차 (2026-10-19 ~ 2026-10-23), 마감 10/22(목) |
| 게이트 | 해당 없음 (다음 주 6주차 = PDR ×2) |
| 저장소 태그 | `week05` (제출 시) |

## 1. 계획 대비 수행 (4주차 평가표 「다음 주 목표」 · 팀장 10/16 회신)

| No. | 계획 | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 1 | assumptions-v0를 **B1 공용 가정값 표 v0.1**로 갱신(월 10/19까지) → 함태훈 URDF와 총질량 · 수평 토크 일치 확인 | (10/16) 팀장 확정값 C1~C8 반영, 공용 json 신설, 평가표 보완점 ①②③ 반영. 함태훈 v0에 공용값 적용 시 74.093 kg = B1 74.096 kg, 수평 토크 22.73 / 37.30 N·m 일치 | 90 % (함태훈 URDF 실제 반영은 함태훈 v1) | b1/models/assumptions-v0.md (v0.1), b1/models/assumptions_b1.json, week05/results/b1_vs_taehoon_mass_torque.md, week05/scripts/compare_b1_taehoon.py |
| 2 | A1 · A2 회신(편차 의도 · RI85 사양 · +z 정면) 반영 v1 재생성 + 체크리스트 재점검 | 회신 대기 — 수령 전까지 가정값 유지 | 0 % | – |
| 3 | 중력보상 토크로 수평 처짐 상쇄 확인 · 캡처 | – | 0 % | – |

## 2. 수치 · 결과

| 항목 | 값 | 근거 |
|---|---|---|
| 총질량 차 74.10 − 67.46 = 6.63 kg | 상판 +3.69 (함태훈 질량 없음) · 몸통 box +3.00 (8 vs 5 kg) · 오른팔 −0.06 (B1 거울 vs CAD) | results/b1_vs_taehoon_mass_torque.md |
| 공용값 적용 후 | 74.093 vs 74.096 kg (B1 더미 프레임 3 × 0.001 kg), 어깨 J1 1.20 m 양쪽 동일 | 〃 |
| 팔 수평 중력토크 | 0 kg 22.73 · 3 kg 37.30 N·m — 두 모델 차 0.005 N·m (팔 관성이 같아 베이스 · 몸통 가정과 무관) | 〃, checklist F3 |
| PDR 판정 기준 | **정격 ≥ 46 N·m** (강 상한 38.4 × 1.2). 통합 모델 37.30 × 1.2 = 44.8 N·m. 피크는 가속분 별도 | assumptions §4 |
| 재계산 (통합 모델) | roll 1.5 rad 외전 3 kg 34.25 N·m (v0 문서 35.2는 B1 임시), elbow +90° 3 kg 7.62 N·m, 스윕 roll 내전 22° · elbow +109° / −108° | assumptions §3 · §4 |
| 공용 json ↔ week04 json | 수치 항목 8개 모두 동일 → v0 모델 재생성 불필요 | 비교 스크립트 출력 |

## 3. 팀장 10/16 결정 반영

- 손목 없음 · **4-DOF 확정** → S13 · C7, 자리표시자 없음 (기획서 6-DOF 불일치는 팀장 상위 보고, 최종 사양 여부 확인 중). 체크리스트 「대상」 6-DOF → 4-DOF 정정
- **C3 평면 3-DOF 채택** → 체크리스트 C3 · C3-1 비고(Isaac Sim 이식 시 3관절 분해) v0.3.1
- **축 부호 규약 안 A(거울) 승인** → assumptions §6, joint-origin-check-v0 §2에 「드라이버 계층 한 곳에서만 반전」 주석
- **PDR 토크 여유 = 정격 기준** → assumptions §4 (effort = 피크는 유지)
- 상판 3.69 kg = 별도 링크 → v1 재생성 때 `base_plate_link`(fixed)로 분리 (v0는 base_link 합성, 질량 · COM 동일)

## 4. 문제 · 해결

- 함태훈 URDF에는 tool0가 없음 → 3 kg 페이로드 점을 B1 left_tool0의 cad_frame 좌표로 옮겨 함태훈 elbow 링크에 고정한 점으로 계산 (두 모델 cad_frame 공통)
- tools/new_week.sh가 CRLF라 VM bash에서 `pipefail` 오류 → log.md 직접 작성 (스크립트 줄바꿈은 그대로 둠)

## 5. 재현

```
python members/park-jongjin/week05/scripts/compare_b1_taehoon.py b1/models/v0/urdf/b1_humanoid_v0.urdf <함태훈 sim/robot/our_robot_v0.urdf (7424a10)> members/park-jongjin/week05/results/b1_vs_taehoon_mass_torque.md
```

## 6. 협업 · 전달

- (→ 함태훈) 공용 가정값 v0.1 적용 요청: 베이스 30 → 30 + 상판 3.69 kg, 몸통 box 5 → 8 kg, torso_joint z 0.8 → 1.20 m(어깨 J1), effort 50 → 100, 관절 범위 ±1.57 → 충돌 스윕 값, damping 0.5, 오른팔 = 왼팔 거울
- (대기) A1 편차 의도 · +z 정면, A2 RI85 사양 — 팀장 경유 전달 완료

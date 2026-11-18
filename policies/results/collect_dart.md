# DART 노이즈 시연 수집 요약 (자동 생성 — `collect_dart_report.py`, week09 박종진)

seed 3000 ~ 3299 300회 → 저장 296 (수치 발산으로 명세 check 실패 → 저장 안 함 4: [3124, 3159, 3225, 3283]), **성공 148** → 학습에 씀(성공 프레임 196,551)
실제 넣은 명령 − 전문가 명령 RMS: 베이스 0.0554 (m/s · rad/s), 팔 0.508°

| 실패 분류 | 개수 |
|---|---|
| approach_timeout | 89 |
| align_fail | 45 |
| ik_fail | 8 |
| table_collision | 5 |
| grasp_miss | 1 |

- 노이즈: OU(상관 시간 0.3 s), 정상상태 σ 베이스 0.05 m/s · 0.08 rad/s, 팔 0.01 rad, 그리퍼 0, 정밀 단계(ALIGN ~ RELEASE) 30 % — `collect_dart.py`
- 처음 σ 팔 0.03 rad에서는 6/6 실패(끝점 ~15 mm > 정렬 허용 6 mm) → 0.01 rad + 정밀 단계 축소
- 실패 대부분이 approach_timeout · align_fail(허용 오차 안으로 못 들어옴) = 노이즈가 계속 들어가서 — 학습에는 성공 에피소드만

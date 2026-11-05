# A2 구동기 case 비교 (자동 생성 — a2_case_compare.py)

게인 = 빌드 모델(week07 고정 kp · kv: pitch 400 · 31 / roll 350 · 27.5 / yaw 300 · 15 / elbow 300 · 11 — week06까지는 kp 300 · dampratio 1), 중력보상 B on, 영점에서 왼팔 관절별 0.2 rad 계단 3 s. I_eff = 영점 질량행렬 대각(다른 관절 서보 고정 근사)

| case | 관절 | armature [kg·m²] | kv | I_eff [kg·m²] | ω_n [rad/s] | 오버슈트 % | 2 % 정착 [s] |
|---|---|---|---|---|---|---|---|
| none (week05, armature 0) | shoulder_pitch | 0.0 | 31.0 | 0.8466 | 21.7 | 0.5 | 0.214 |
| none (week05, armature 0) | shoulder_roll | 0.0 | 27.5 | 0.737 | 21.8 | 0.23 | 0.208 |
| none (week05, armature 0) | shoulder_yaw | 0.0 | 15.0 | 0.0153 | 140.0 | 0.0 | 0.194 |
| none (week05, armature 0) | elbow | 0.0 | 11.0 | 0.0605 | 70.4 | 0.0 | 0.124 |
| a2_doc | shoulder_pitch | 0.219 | 31.0 | 1.0656 | 19.4 | 1.56 | 0.192 |
| a2_doc | shoulder_roll | 0.219 | 27.5 | 0.956 | 19.1 | 1.92 | 0.184 |
| a2_doc | shoulder_yaw | 0.219 | 15.0 | 0.2343 | 35.8 | 0.18 | 0.132 |
| a2_doc | elbow | 0.098 | 11.0 | 0.1585 | 43.5 | 1.35 | 0.094 |
| cad_ri85 | shoulder_pitch | 0.098 | 31.0 | 0.9446 | 20.6 | 0.85 | 0.202 |
| cad_ri85 | shoulder_roll | 0.098 | 27.5 | 0.835 | 20.5 | 0.81 | 0.194 |
| cad_ri85 | shoulder_yaw | 0.098 | 15.0 | 0.1133 | 51.5 | 0.0 | 0.172 |
| cad_ri85 | elbow | 0.098 | 11.0 | 0.1585 | 43.5 | 1.43 | 0.094 |

- none (week05, armature 0): 팔 앞으로 수평 5 s 유지 오차 0.0°
- a2_doc: 팔 앞으로 수평 5 s 유지 오차 0.0°
- cad_ri85: 팔 앞으로 수평 5 s 유지 오차 0.0°
- armature가 가장 가벼운 관절(yaw · elbow)의 I_eff를 몇 배로 키움 → ω_n이 크게 내려가고 정착이 느려짐(함태훈 armature_study.md 예측과 같은 방향). 로터만 값이라 실제는 더 큼(기어열 TBD)
- kp · kv 재튜닝(함태훈 6주차 과제)은 a2_doc 기준으로 할 것을 제안 — 어깨 TQ/RI85 확정 시 case만 바꾸고 다시 빌드

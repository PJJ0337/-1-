# A2 구동기 case 비교 (자동 생성 — a2_case_compare.py)

kp 300 · dampratio 1(kv 재계산), 중력보상 B on, 영점에서 왼팔 관절별 0.2 rad 계단 3 s. I_eff = 영점 질량행렬 대각(다른 관절 서보 고정 근사)

| case | 관절 | armature [kg·m²] | kv | I_eff [kg·m²] | ω_n [rad/s] | 오버슈트 % | 2 % 정착 [s] |
|---|---|---|---|---|---|---|---|
| none (week05, armature 0) | shoulder_pitch | 0.0 | 20.531 | 0.8466 | 18.8 | 3.68 | 0.316 |
| none (week05, armature 0) | shoulder_roll | 0.0 | 28.058 | 0.737 | 20.2 | 0.05 | 0.272 |
| none (week05, armature 0) | shoulder_yaw | 0.0 | 4.251 | 0.0153 | 140.0 | 0.01 | 0.044 |
| none (week05, armature 0) | elbow | 0.0 | 6.244 | 0.0605 | 70.4 | 1.61 | 0.062 |
| a2_doc | shoulder_pitch | 0.219 | 29.791 | 1.0656 | 16.8 | 0.6 | 0.266 |
| a2_doc | shoulder_roll | 0.219 | 32.448 | 0.956 | 17.7 | 0.04 | 0.316 |
| a2_doc | shoulder_yaw | 0.219 | 16.761 | 0.2343 | 35.8 | 0.0 | 0.164 |
| a2_doc | elbow | 0.098 | 12.823 | 0.1585 | 43.5 | 0.28 | 0.122 |
| cad_ri85 | shoulder_pitch | 0.098 | 27.235 | 0.9446 | 17.8 | 0.77 | 0.24 |
| cad_ri85 | shoulder_roll | 0.098 | 30.109 | 0.835 | 19.0 | 0.05 | 0.294 |
| cad_ri85 | shoulder_yaw | 0.098 | 11.649 | 0.1133 | 51.5 | 0.0 | 0.116 |
| cad_ri85 | elbow | 0.098 | 12.678 | 0.1585 | 43.5 | 0.36 | 0.118 |

- none (week05, armature 0): 팔 앞으로 수평 5 s 유지 오차 0.0°
- a2_doc: 팔 앞으로 수평 5 s 유지 오차 0.0°
- cad_ri85: 팔 앞으로 수평 5 s 유지 오차 0.0°
- armature가 가장 가벼운 관절(yaw · elbow)의 I_eff를 몇 배로 키움 → ω_n이 크게 내려가고 정착이 느려짐(함태훈 armature_study.md 예측과 같은 방향). 로터만 값이라 실제는 더 큼(기어열 TBD)
- kp · kv 재튜닝(함태훈 6주차 과제)은 a2_doc 기준으로 할 것을 제안 — 어깨 TQ/RI85 확정 시 case만 바꾸고 다시 빌드

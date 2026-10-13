# 체크리스트 v0.3 점검 — b1_humanoid_v0 (MuJoCo 3.15.0)

자동 생성: v0_checklist_check.py · 합계 25항목 — OK 25 · NG 0 

| No. | 점검 | 결과 | 근거 |
|---|---|---|---|
| A1 | 모든 link에 inertial | OK | 없음: - (odom = 월드 고정 루트, 질량 불필요) |
| A2 | 질량 ≥ 1e-3 (가상 링크는 fixed) | OK | tool0 2개 = 0.001 kg · fixed |
| A3 | 관성 양정치 · 삼각부등식 | OK | 위반 0개 — balanceinertia 미사용(B6) |
| A4 | 단위 m (메시 최대 길이) | OK | 최대 extent 0.372 m (mm 그대로면 ~300) |
| A5 | revolute 8개 limit(lower·upper·effort·velocity) | OK | revolute 8개, 누락 0 — 값은 가정(assumptions-v0) |
| A6 | 트리(폐루프 없음) | OK | child 중복 없음 |
| A8 | transmission · gazebo 미사용 | OK | 액추에이터는 robot_actuated.xml |
| B1 | fusestatic=false | OK | torso_link(fixed) · tool0 이름 보존 |
| B2 | discardvisual=false | OK |  |
| B4 | meshdir 지정 | OK | strippath 기본 → 파일명만 |
| B6 | balanceinertia=false (팀 결정 10/8) | OK |  |
| B3 | robot.xml angle=radian | OK | mj_saveLastXML이 명시 |
| F1 | body 수 · 이름 = URDF link | OK | nbody 14 = link 13 + world, njnt 11, ngeom 24 |
| C3 | 베이스 planar → slide·slide·hinge | OK | ['base_planar_joint_TX', 'base_planar_joint_TY', 'base_planar_joint_RZ'], nq 11 = 3 + 8 |
| D1 | 영점 자세 접촉 0 | OK | ncon 0 |
| D3 | 관절 범위 양 끝 접촉 0 (단독 구동) | OK | 16자세 접촉 합 0 — 범위는 충돌 스윕 기준(assumptions-v0 §2) |
| D4 | visual geom contype=conaffinity=0 | OK | URDF visual → group 1 |
| E1 | 3단 파일 구조 | OK | robot.xml(변환본) → robot_actuated.xml → scene.xml |
| E4 | forcerange = effort, ctrlrange = 관절 range | OK | forcerange ±100 Nm, inheritrange=1 |
| E5 | implicitfast + dampratio | OK | dampratio=1 (임계감쇠) |
| E6 | armature (BOM 전 0 허용) | OK | 0 — RI85 사양(J_rotor·N²) 수령 시 교체, ICD 요청서 |
| F3 | 수평 자세 중력토크 손계산 = MuJoCo | OK | 손계산 Σ r×mg = 23.8188 N·m, MuJoCo 23.8188 N·m |
| F2 | 총질량 ≤ 80 kg (기획서) | OK | 75.80 kg (베이스 33.7 · 몸통 box 8 kg 가정 포함) |
| E7 | position 서보 처짐 ≈ τg/kp | OK | 시뮬 0.0791 rad vs 이론 0.0794 rad (kp 300) |
| F5 | 뷰어/캡처: 영점 · 범위 · 중력 | OK | 캡처 4자세, 영점 유지 2 s 처짐 0.0004 rad |

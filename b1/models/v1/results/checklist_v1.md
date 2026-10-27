# 체크리스트 v0.4 점검 — b1_v1 (MuJoCo 3.15.0)

자동 생성: checklist_v1_check.py · 합계 28항목 — OK 28 · NG 0 

| No. | 점검 | 결과 | 근거 |
|---|---|---|---|
| A1 | 모든 link에 inertial | OK | 없음: - (odom = 월드 고정 루트) |
| A2 | 질량 ≥ 1e-3, 가상 링크는 fixed | OK | 1 g 가상 링크 3개(cad_frame · tool0 ×2) 모두 fixed. Isaac 사본의 base_x · base_y_link는 움직이는 가상 링크라 1 g 필수 |
| A3 | 관성 양정치 · 삼각부등식 (오른팔 미러 관성 포함) | OK | 위반 0 — balanceinertia 미사용(B6) |
| A4 | 단위 m (메시 최대 길이) | OK | 최대 extent 0.372 m |
| A5 | revolute 8개 limit(lower·upper·effort·velocity) | OK | revolute 8, 누락 0 — 값 = assumptions_b1.json (공용 v0.2) |
| A6 | 트리(폐루프 없음) | OK | child 중복 없음 |
| A8 | transmission · gazebo 미사용 | OK | 액추에이터는 b1.xml(빌드 덧붙이기) |
| B1 | fusestatic=false | OK | base_plate_link · torso · tool0 · gripper_link 이름 보존 → site · sensor 대상 |
| B2 | discardvisual=false | OK |  |
| B4 | 메시 상대 경로 (../meshes/visual · collision, package:// 없음) | OK | 18개, strippath=false — Isaac 임포트 패키지 요건 (week06: meshdir → 파일별 상대 경로) |
| B6 | balanceinertia=false (팀 결정 10/8) | OK |  |
| B3 | 변환본 angle=radian | OK | mj_saveLastXML이 명시 |
| F1 | body 수 · 이름 = URDF link | OK | nbody 18 = link 17 + world, njnt 11, ngeom 29, nsite 3 |
| C3 | 베이스 planar → slide · slide · hinge (원본) | OK | ['base_planar_joint_TX', 'base_planar_joint_TY', 'base_planar_joint_RZ'], nq 11 = 3 + 8 |
| C3-1 | Isaac 사본: prismatic · prismatic · continuous(z) + 가상 링크 2 | OK | [('base_x_joint', 'prismatic'), ('base_y_joint', 'prismatic'), ('base_yaw_joint', 'continuous')], 원본과 50자세 차 5.3e-15 |
| D1 | 영점 자세 접촉 0 (바닥 제외) | OK | ncon 0 |
| D3 | 관절 범위 sweep 자기 충돌 0 (검증 ④) | OK | 8관절 × 0.5° 3770자세, 접촉 0. 한계 밖 여유 최소: roll − 2° · elbow ± 3~4° |
| D4 | visual geom contype=conaffinity=0 | OK | URDF visual → group 1 |
| E1 | 파일 구조: 변환본 · 빌드 생성 b1.xml · scene | OK | b1_urdf.xml(수정 금지) → b1.xml(build_v1.py + actuator_params.json) → scene.xml (0-3절) |
| E4 | forcerange = effort, ctrlrange = 관절 range | OK | ±[181.5, 381.7] N·m (A2 피크), inheritrange=1, class shoulder · elbow |
| E5 | implicitfast + dampratio | OK | dampratio 1 → kv는 컴파일 시 영점 관성으로 계산 (관성 바뀌면 자동 갱신) |
| E6 | armature = A2 사양 (J_rotor × i², week06) | OK | case a2_doc: [0.219, 0.219, 0.219, 0.098] (pitch · roll · yaw · elbow, 양팔 같음) — 로터만, 기어열 관성 TBD(A2) → 과소 추정 |
| E8 | 액추에이터 11개 명령 → 관절 응답 (뷰어 슬라이더 점검 자동판) | OK | 11/11 — 범위 40 % 명령, position 오차 ≤ 2 %, 베이스 velocity 1.5 s 후 98 % |
| F3 | 수평 자세 중력토크 손계산 = MuJoCo | OK | 손계산 Σ r×mg = 25.4035 N·m, MuJoCo 25.4035 N·m (그리퍼 0.5 kg 포함, v0 22.7301) |
| F2 | 총질량 ≤ 80 kg (기획서) · 검증 ① | OK | 75.097 kg, 기준 대비 +0.004 % |
| E7 | position 서보 처짐 ≈ τg/kp · 중력보상으로 상쇄 | OK | 보상 없음 4.8324° vs 이론 4.852° → 보상(목표 + τg/kp) 0.0° |
| M1 | 좌우 미러 = 규칙표 · 검증 ⑤ | OK | 504자세 |Δx|·|Δz|·|y_L+y_R| ≤ 0.0e+00 mm — docs/mirror_rules.md |
| F5 | 캡처: 4자세 · collision · 중력보상 | OK | img/v1_poses.png · v1_collision.png · v1_gravcomp.png + 노트북 뷰어 재현 img/repro/20261016_laptop_v1_*.png (Control 슬라이더 · 양팔 대칭 · base_vx) |

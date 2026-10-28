# b1_v1 — 양팔 + 홀로노믹 베이스 전체 모델 (week05 → week06: 기구학 · 작업공간 · IK · Isaac 임포트 패키지)

v0(week04)를 커리큘럼 5주차 「핵심 설계 결정」대로 확장. **URDF가 원본**이고 MJCF는 빌드가 URDF에서 만든다. 값은 [../assumptions_b1.json](../assumptions_b1.json)(B1 공용 v0.3)과 [actuator_params.json](actuator_params.json)(MJCF 덧붙이기, 함태훈 담당), [inputs/a2_actuators.json](inputs/a2_actuators.json)(A2 구동기 사양 10/19, week06) 세 곳뿐.

```
v1/
├── actuator_params.json        MJCF 덧붙이기 값 (class별 kp · dampratio · forcerange · armature · damping, 베이스 kv, 센서)
├── inputs/link_inertials_v1.json  링크 질량 · 관성 (함태훈 13b01ad, 원점 = 골격 관절점)
├── inputs/a2_actuators.json    A2 사양 → case a2_doc(어깨 TQ + 팔꿈치 RI85, 기본) · cad_ri85(8개 RI85): armature · 피크(forcerange · URDF effort)
├── urdf/b1_v1.urdf             원본: odom →planar→ base_link →fixed→ base_plate_link · torso_link → 양팔 4관절 → tool0 → gripper_link
├── urdf/b1_v1_isaac.urdf       Isaac Sim용: planar → base_x(prismatic) · base_y(prismatic) · base_yaw(continuous) + 가상 링크 2
├── meshes/visual/*.stl · meshes/collision/*.stl   링크 프레임 · m 단위 (week06: 하위 폴더 분리, URDF는 ../meshes/… 상대 경로 · strippath=false)
├── mjcf/b1_urdf.xml            URDF 변환본 (mj_saveLastXML, 수정 금지)
├── mjcf/b1.xml                 변환본 + site(left_ee · right_ee · base_imu) + default class + 액추에이터 11 + 센서 30
├── mjcf/scene.xml              include b1.xml + 바닥 · 조명 · option
├── docs/mirror_rules.md        좌우 미러 규칙표 (자동 생성)
├── docs/ik_interface.md        IK 함수 인터페이스 (함태훈 집기 시나리오용, week06 v1)
├── docs/import_settings.md     MuJoCo ↔ Isaac Sim URDF Importer ↔ Isaac Lab 대응표 + 실습 PC 첫날 할 일 (자동 생성)
├── docs/actuator_params.md     관절별 Isaac Lab ImplicitActuatorCfg 값 (함태훈 week06 사본, 원본 sim/w06/docs)
├── isaac/b1_v1_cfg.py          Isaac Lab ArticulationCfg 초안 (자동 생성)
├── scripts/                    build_v1 · validate_model · base_velocity_test · render_v1 · checklist_v1_check · v1_mjio
│                               + week06: workspace · ik · ik_demo · a2_case_compare · export_isaac
├── results/                    validate_model.md(검증 5종) · checklist_v1.md · base_velocity_test · actuator_check · gravcomp
│                               + week06: workspace.md · workspace_max_torque.json · ik_results.md · a2_case_compare.md · isaac_package_check.json
└── img/                        v1_poses · v1_collision · v1_gravcomp · base_velocity_test + week06: workspace.png · ik_targets.png · ik_targets.gif
```

| 구성 | 값 |
|---|---|
| 자유도 | nq = nv = 11 (베이스 평면 3 + 팔 4 × 2), nu = 11 (velocity 3 + position 8), 센서 30 (jointpos · jointvel · actuatorfrc × 8, IMU gyro · accelerometer, ee framepos · framequat × 2) |
| 총질량 | **75.10 kg** = v0 74.10 + 그리퍼 임시 box 0.5 kg × 2 (S14, BOM 대기) |
| 이름 | ICD 규칙 `{side}_{부위}_{운동}` 유지 (커리큘럼 예시의 `r_*` · `l_*` · `r_ee` = 우리 `right_*` · `left_*` · `right_ee`). 기준 팔 = left |
| site | `{side}_ee` = 그리퍼 box 끝 (tool0에서 0.10 m), `base_imu` = 상판 중심 |

## 실행

```bash
pip install mujoco==3.15.0 trimesh scipy matplotlib
python b1/models/v1/scripts/build_v1.py                     # URDF 2종 → 변환본 → b1.xml · scene.xml, 미러 규칙표
python b1/models/v1/scripts/validate_model.py               # 검증 5종 → results/validate_model.md
python b1/models/v1/scripts/base_velocity_test.py           # 베이스 속도 명령 vs qvel
python b1/models/v1/scripts/render_v1.py                    # 캡처 · 액추에이터 응답 · 중력보상 (리눅스 헤드리스: MUJOCO_GL=osmesa)
python b1/models/v1/scripts/checklist_v1_check.py           # 체크리스트 v0.4 → results/checklist_v1.md
python b1/models/v1/scripts/v1_mjio.py b1/models/v1/mjcf/scene.xml   # 뷰어 (Windows 한글 경로 대응)
# week06
python b1/models/v1/scripts/workspace.py                    # 작업공간 점구름(팔당 30,000) · 공유 · 높이별 도달 · 토크 최대 자세 (~20 s)
python b1/models/v1/scripts/ik.py                           # IK 목표 10개 · λ 비교 · 수렴률 500 (~6 s)
python b1/models/v1/scripts/ik_demo.py                      # IK GIF (리눅스: MUJOCO_GL=glfw xvfb-run -a …)
python b1/models/v1/scripts/a2_case_compare.py              # A2 armature case별 계단 응답
python b1/models/v1/scripts/export_isaac.py                 # Isaac 패키지 점검 + import_settings.md + isaac/b1_v1_cfg.py
```

week06 빌드 기본값: **중력보상 B on**(`actuator_params.json` gravcomp.enable), **A2 case `a2_doc`**(어깨 armature 0.219 · 피크 381.7, 팔꿈치 0.098 · 181.5). case 전환 = `actuator_params.json` `a2.case` 또는 `B1_A2_CASE=cad_ri85 python build_v1.py`. 검증 ②의 「보상 없음 · A」는 `load_model(..., strip_gravcomp=True)`로 계산

## 알려진 한계

- 옴니휠 미모델링(평면 3-DOF 근사) — 바퀴 슬립 · 접촉력 · 전복 없음 (구술 노트 [04](../../../members/park-jongjin/notes/mujoco-urdf/04-v1-mirror-base.md))
- 그리퍼 = 0.5 kg box 가정(S14), armature = A2 로터만(기어열 TBD → 과소), 어깨 구동기 TQ/RI85 팀장 확인 대기, 관절 범위 = 충돌 스윕 + 케이블 가정, 하체 · 몸통 = 가정값
- 4-DOF(손목 없음): 위치 + 그리퍼 축 동시 지정은 2차원 곡면에서만 가능(IK `orientation_limit`), 최저 도달 z 0.606 m → 상판 윗면 ~ 0.206 m 위는 도달 불가
- 오른팔 = 공칭 거울 — CAD 오른팔 편차(축 1.37°, 위치 7.4 mm)는 [joint-origin-check-v0.md](../joint-origin-check-v0.md)
- 함태훈 v1 메시 · 관성(골격 관절점 원점) 수령 시 `link_inertials` 입력만 바꾸고 다시 빌드

# week02 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 2 주차 |
| 게이트 | **SRR (×2)** — 통과 기준: 하체 실측 완료 · 목표 사양 확정 · ICD v0 발행 |
| 제출일 | 2026-10-01(목) — 담당 대학원생(김이겸)에게 제출 완료 |
| 저장소 태그 | `week02` (제출 커밋에 부여) |
| 주간 보고서 | [reports/학부생_주간활동보고서_2주차_박종진_22212289_20261001.docx](../reports/학부생_주간활동보고서_2주차_박종진_22212289_20261001.docx) |

> **실습 PC(Ubuntu · RTX GPU) 미배정** → 1~3번 대신 김이겸 대학원생의 「B1 2·3주차 학습 커리큘럼 — MuJoCo · URDF · USD (GPU 없이)」(Notion)을 따름.
> 5~7번은 2026-09-29(2주차 화) 수행 — 스크립트 · 초안 작성에 Claude(AI) 보조 사용, G1 뷰어 조작은 본인 노트북에서 직접 수행. 분석 스크립트 3종(urdf_check · oral_experiments · g1_inspect)을 **본인 노트북(Windows, Python 3.12)에서 재실행해 같은 결과 확인**([캡처](img/repro/)).

## 1. 계획 대비 수행

1주차 평가표 「다음 주 목표」 + B1 2주차 커리큘럼(MuJoCo · URDF, GPU 없이) 기준. 1~3번은 실습 PC 배정을 전제로 한 목표라 커리큘럼(5~7번)으로 대체.

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | 실습 PC 배정 시 Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 → create_empty.py · Isaac-Ant-v0 headless 스크린샷 | 미착수 — 실습 PC 미배정 | – (보류) | 4주차 이후 환경 확보 시 |
| 2 | Cartpole zero_agent · random_agent(`--num_envs 32`) 직접 실행 | 미착수 — 실습 PC 미배정 | – (보류) | 4주차 이후 환경 확보 시 |
| 3 | (PC 미배정 시) H1 locomotion env cfg · RewTerm · ObsTerm 정독 → 관측·보상·종료 구조 문서화, 2.3.x ActuatorCfg/SensorCfg 변경 인자 정리 | 대학원생 커리큘럼(GPU 없이 MuJoCo · URDF · USD)으로 대체 — 평가 보완점 ③(RL보다 모델 구축 비중 확대)과 같은 방향 | – (대체) | 5~7번 |
| 4 | Git 저장소 개인 폴더 생성, 노션 정리본을 md로 옮겨 `week02` 태그로 첫 기여 로그 업로드 | 저장소 구조·규칙 수립([README](../../../README.md), [CONTRIBUTING](../../../CONTRIBUTING.md), [템플릿](../../../templates/contribution_log.md), [new_week.sh](../../../tools/new_week.sh)). 노션 페이지 12건(설치 · 데모 · Core Concepts 9항목)을 md 3건으로 이전, 원문 링크·원작성자 표기 | 100 % | [notes/isaac-lab/](../notes/isaac-lab/), 태그 `week02` |
| 5 | (커리큘럼 개별 과제) G1 MJCF 전체 구조 트리 + 우리 로봇(6-DOF 팔 2 + 홀로노믹 베이스) 맞춤 변경 목록 | G1 바디 트리 · 요약 수치 · 팔 관절 표 작성, 변경 항목 13개 목록화 (루트 · 하체 · 팔 7→6-DOF · 액추에이터 · 센서 등) | 100 % | [g1-structure-tree.md](g1-structure-tree.md), [img/g1_tree.png](img/g1_tree.png)(트리 그림), [scripts/g1_inspect.py](scripts/g1_inspect.py), [scripts/g1_tree_diagram.py](scripts/g1_tree_diagram.py), [img/g1_body_tree.txt](img/g1_body_tree.txt), [img/g1_arm_joints.md](img/g1_arm_joints.md), [img/g1_default_vs_moved.png](img/g1_default_vs_moved.png) |
| 6 | (SRR 기여) A팀 요청 항목 초안: CAD 포맷, 질량·관성 제공 형식, 관절 좌표계 규약, 링크 이름 규칙 | ICD v0 요청 초안 7개 절 (+ 구동기 사양 · 센서 위치 · 변경 관리) | 100 % (대학원생 검토 전) | [b1/docs/icd/icd-v0-request-draft.md](../../../b1/docs/icd/icd-v0-request-draft.md) |
| 7 | (공통 월~목) MuJoCo 설치 · G1 조작, MJCF 구조, 2링크 URDF 작성, MuJoCo로 열어 검증 | **본인 Windows 노트북에서 MuJoCo 뷰어로 G1 실행 · Control 슬라이더로 팔 관절 조작(2026-09-29)**. 2링크 URDF 손으로 작성 → MuJoCo 로드 성공, 질량 · FK · 중력 토크 해석해와 일치, fixed 링크 소실(fusestatic) 발견. **본인 노트북 뷰어로 열어 베이스-link1 충돌로 어깨가 안 떨어지는 문제 발견 → URDF 수정** | 100 % | [img/20260929_g1_viewer_default.png](img/20260929_g1_viewer_default.png), [img/20260929_g1_viewer_arm_moved.png](img/20260929_g1_viewer_arm_moved.png), [img/20260929_g1_viewer_joint_pose_paused.png](img/20260929_g1_viewer_joint_pose_paused.png), [img/20260929_two_link_viewer_before_fix.png](img/20260929_two_link_viewer_before_fix.png), [img/20260929_two_link_viewer_after_fix.png](img/20260929_two_link_viewer_after_fix.png), [models/two_link_arm.urdf](models/two_link_arm.urdf), [scripts/urdf_check.py](scripts/urdf_check.py), [img/two_link_arm_mujoco.png](img/two_link_arm_mujoco.png), [notes/mujoco-urdf/01-mjcf-urdf-basics.md](../notes/mujoco-urdf/01-mjcf-urdf-basics.md) |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| G1 규모 | 바디 31 · 관절 30(free 1 + hinge 29) · nu 29 · 33.34 kg | g1_inspect.py |
| G1 팔 | 7-DOF, 어깨 · 팔꿈치 ±25 Nm, 손목 pitch/yaw ±5 Nm, gear 1, position kp 500 | g1.xml, g1_arm_joints.md |
| 우리 사양(잠정) | 팔 6-DOF × 2, 어깨 100 Nm급 · 팔꿈치 40 Nm급, ≤ 80 kg | 기획서 목표 사양 |
| 2링크 수평 중력 토크 | shoulder 3.9731 Nm, elbow 0.7358 Nm (해석해 = MuJoCo) | urdf_check.py |
| inertial 생략 | geom × 1000 kg/m³ 추정 (0.2×0.1×0.1 상자 → 2.0 kg), 질량 0 가동 body는 컴파일 오류 | oral_experiments.py |
| URDF fixed 링크 | MuJoCo 기본 로드 시 합쳐져 사라짐 (body 5 → 3) | urdf_check.py |
| 2링크 토크 0 낙하 (수정 전 → 후) | 3초 후 shoulder 0.0° · elbow 90.3° (접촉 1개) → shoulder 90.0° · elbow 1.3° (접촉 0개, 하한 +90°에서 정지) | 뷰어 캡처, urdf_check.py |

## 3. 재현 방법

```bash
# MuJoCo 환경 (Windows, GPU 불필요) — 검증: mujoco 3.14.0, 본인 노트북 Python 3.12에서 재현 확인(2026-09-29)
pip install -r requirements.txt          # 저장소 루트
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie   # 저장소 루트에 (gitignore됨)
python -m mujoco.viewer --mjcf=mujoco_menagerie/unitree_g1/scene.xml   # 월: 관절 드래그 → 스크린샷 img/

cd members/park-jongjin/week02
python scripts/g1_inspect.py ../../../mujoco_menagerie img   # 화: 트리 · 관절 표 · 렌더
python scripts/urdf_check.py models/two_link_arm.urdf img       # 목: URDF 검증
python scripts/oral_experiments.py                              # 구술 Q1 · Q2 실험
```

## 4. 발견한 문제 · 해결

- **2링크 팔을 뷰어로 열었더니 어깨가 수평(−0.014 rad)에서 멈추고 팔꿈치만 90°로 처짐**([캡처](img/20260929_two_link_viewer_before_fix.png)). 원인: 어깨 관절이 베이스 원통(높이 0.10) 바로 위 z=0.10에 있어 link1(반지름 0.03) 아랫면이 베이스와 0.03 m 겹침 → 접촉 1개가 어깨를 받침. MuJoCo는 월드에 고정된 부모(base_link)와 자식(link1)의 접촉을 걸러내지 않음(fusestatic=false로 base_link를 살려도 동일). 해결: 고정 베이스라 바닥 충돌이 필요 없고 인접 링크 충돌은 보통 끄므로(Isaac Sim Self-Collision 기본 off) **base_link의 collision 제거** → 접촉 0개, 어깨가 하한 +90°까지 떨어짐. 교훈: 관절 축 근처에서 인접 링크 충돌 형상이 겹치지 않게 설계하거나 충돌 쌍을 제외해야 함(임포트 체크리스트 반영). 수정 후 뷰어: shoulder 1.57 rad · elbow ≈ 0 ([캡처](img/20260929_two_link_viewer_after_fix.png)).
- 수정 후 뷰어에서 **베이스 원통이 안 보임** → MuJoCo는 URDF를 읽을 때 `<visual>`을 기본으로 버림(`discardvisual` 기본 true, 실험: geom 2개 / `discardvisual="false"` 시 5개). 화면에 보이던 원통은 collision 형상이었음. 시각 확인이 필요하면 `<mujoco><compiler discardvisual="false"/></mujoco>` 추가.
- MuJoCo는 URDF를 읽을 때 fixed joint로만 붙은 링크(base_link · tool0)를 부모에 합친다(`fusestatic` 기본 true) → 센서 · 툴 프레임이 사라짐. URDF에 `<mujoco><compiler fusestatic="false"/></mujoco>` 를 넣어 보존. Isaac Sim merge fixed joints와 같은 문제라 3주차 체크리스트 항목으로 연결.
- `<inertial>` 누락 시 MuJoCo가 오류 없이 밀도 1000으로 추정 → A팀 관성 데이터 누락을 모델에서 알아채기 어려움 → ICD에 "관성 누락 금지 · 출력 좌표계 표기" 추가.
- 뷰어에서 실행 중(Run) `left_shoulder_pitch` 목표를 −3.09 rad(하한 근처)까지, `left_elbow` 0.209 rad로 주자 **G1이 뒤로 넘어짐**([사진](img/20260929_g1_viewer_arm_moved.png)). G1은 pelvis가 freejoint(떠 있는 베이스)이고 액추에이터는 관절 목표각만 유지하는 position(kp 500)이라 균형 제어가 없음 → 팔을 크게 휘두르면 무게중심 이동을 보상하지 못함. 우리 로봇은 베이스가 바닥에 있지만 기획서 리스크 「상체 하중에 의한 전도 모멘트」와 같은 문제 → 모델 검증 시 팔 자세별 무게중심 확인 필요.
- 비교: **일시정지(PAUSE) 상태에서 Joint 패널로** 자세를 바꾸면 물리 계산 없이 관절각(qpos)만 바뀌어 넘어지지 않음 — 왼팔 들기(left_shoulder_pitch −1.3, roll 0.696, yaw −0.236), 오른팔 벌리기(right_shoulder_pitch 0.453, yaw −0.733) 자세를 직접 만들어 확인([캡처](img/20260929_g1_viewer_joint_pose_paused.png)). Control 패널은 액추에이터 목표값(ctrl)을 바꾸고 시뮬레이션이 돌며 따라가는 방식이라 동역학(전도)이 드러남.
- G1 팔은 7-DOF라 우리 6-DOF와 구성이 다름 → 어느 손목 축을 뺄지 설계안 확인 필요(ICD 확인 항목).

## 5. 막힌 점 · 요청

- 실습 PC(Ubuntu · RTX GPU) 미배정 — Isaac Sim 설치 · 데모 직접 실행(1 · 2번) 보류. 배정 시 즉시 착수.
- 1주차 질문(학습 문서 v2.1.0 ↔ 설치 대상 v2.3.2 의 ActuatorCfg/SensorCfg 차이) 회신 대기.

## 6. 구술 질문 준비 (커리큘럼 지정)

- Q1. MJCF에서 body와 joint의 부모-자식 관계가 관절 좌표계를 어떻게 정하는가?
  - 답변 요지: 자식 body의 pos/quat이 부모 기준으로 자식 좌표계를 정하고, joint의 pos/axis는 그 자식 body 좌표계로 해석된다. 실험: 같은 axis (0,1,0)도 자식 body를 z축 90° 돌리면 world 축이 (−1,0,0)이 되고, joint pos는 앵커만 옮긴다. → [상세](../notes/mujoco-urdf/01-mjcf-urdf-basics.md#q1-mjcf에서-body와-joint의-부모-자식-관계가-관절-좌표계를-어떻게-정하는가)
- Q2. inertial 태그를 생략하면 MuJoCo는 어떻게 처리하는가?
  - 답변 요지: `inertiafromgeom=auto`(기본)이면 geom 부피 × 밀도(기본 1000 kg/m³)로 계산(0.2×0.1×0.1 상자 → 2.0 kg). inertial이 있으면 그것이 우선, `true`면 geom으로 덮어씀. 가동 body에 질량이 없으면(geom 없음 · density=0 · `false`) `mass and inertia of moving bodies must be larger than mjMINVAL` 컴파일 오류. → [상세](../notes/mujoco-urdf/01-mjcf-urdf-basics.md#q2-inertial-태그를-생략하면-mujoco는-어떻게-처리하는가)

## 7. 다음 주 계획

[week03/log.md](../week03/log.md) 참고.

# 박종진 — 개인 기여 로그

| 항목 | 내용 |
|---|---|
| 학번 | 22212289 |
| 팀 / 서브팀 | TEAM B / B1 (모델링 · 시뮬레이션) |
| 담당 대학원생 | 김이겸 |
| 담당 업무 | 로봇 모델 구조 분석(MJCF/URDF), URDF 임포트 체크리스트, A팀 대상 ICD 요청 항목 |

## 주차별 로그

| 주차 | 게이트 | 태그 | 로그 | 주요 산출물 | 보고서 |
|---|---|---|---|---|---|
| 1 | – | – | [week01/log.md](week01/log.md) | [notes/isaac-lab/](notes/isaac-lab/) (노션 원본 이전) | [1주차 보고서](reports/) |
| 2 | SRR ×2 | `week02` | [week02/log.md](week02/log.md) | 저장소 개설 · 노션 이전, [G1 구조 트리 · 변경 목록](week02/g1-structure-tree.md), [2링크 URDF](week02/models/two_link_arm.urdf), [ICD v0 요청 초안](../../b1/docs/icd/icd-v0-request-draft.md) | [2주차 보고서](reports/학부생_주간활동보고서_2주차_박종진_22212289_20261001.docx) |
| 3 | – | `week03` | [week03/log.md](week03/log.md) | [2링크 MJCF(include · default)](week03/mjcf/), [계단 응답](week03/img/20261006_step_all.png), [MuJoCo 로딩 체크리스트](../../b1/docs/procedures/mujoco-urdf-loading-checklist.md), [파이프라인 그림](week03/img/pipeline_mujoco.png), USD 선행 실습(부록) | [3주차 보고서](reports/학부생_주간활동보고서_3주차_박종진_22212289_20261008.docx) |
| 4 | – | `week04` | [week04/log.md](week04/log.md) | v0 URDF · 링크 분할 · 가정값, [notes 03](notes/mujoco-urdf/03-cad-to-urdf.md) | [4주차 보고서](reports/학부생_주간활동보고서_4주차_박종진_22212289_20261015.docx) |
| 5 | – | `week05` | [week05/log.md](week05/log.md) | v1 MJCF · 검증 5종, [notes 04](notes/mujoco-urdf/04-v1-mirror-base.md) | [5주차 보고서](reports/학부생_주간활동보고서_5주차_박종진_22212289_20261022.docx) |
| 6 | PDR ×2 | `week06` | [week06/log.md](week06/log.md) | 작업공간 · IK · Isaac 패키지, [notes 05](notes/mujoco-urdf/05-workspace-ik.md) | [6주차 보고서](reports/학부생_주간활동보고서_6주차_박종진_22212289_20261029.docx) |
| 7 | – | `week07` | [week07/log.md](week07/log.md) | 전신 IK · 작업공간 확장 · 토크 예산, [notes 06](notes/mujoco-urdf/06-wholebody-ik.md) | [7주차 보고서](reports/학부생_주간활동보고서_7주차_박종진_22212289_20261105.docx) |
| 8 | – | `week08` | [week08/log.md](week08/log.md) | [scenarios/](../../scenarios/README.md) 집기 상태기계 · [pick20](../../scenarios/results/pick20.md) · [수집 · 재생](../../scenarios/results/collect.md) · [그리퍼](../../scenarios/results/gripper_slip.md), [notes 07](notes/mujoco-urdf/07-contact-grasp.md) | [8주차 보고서](reports/학부생_주간활동보고서_8주차_박종진_22212289_20261112.docx) |

## 학습 정리 (notes)

| 파일 | 내용 | 원 자료 |
|---|---|---|
| [notes/isaac-lab/01-install.md](notes/isaac-lab/01-install.md) | Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 5단계 · 검증 명령 | 김이겸, Notion「캡스톤디자인(1)」— 개발환경 구축 |
| [notes/isaac-lab/02-demos.md](notes/isaac-lab/02-demos.md) | list_envs · showroom · zero/random agent · H1 locomotion 실행 명령 | 동 — Isaac Lab 기초 데모 |
| [notes/isaac-lab/03-core-concepts.md](notes/isaac-lab/03-core-concepts.md) | Task Workflows · Actuators · Sensors 5종 · Motion Generators + B1 적용 지점 | 동 — Core Concepts |
| [notes/mujoco-urdf/01-mjcf-urdf-basics.md](notes/mujoco-urdf/01-mjcf-urdf-basics.md) | MJCF · URDF 구조, 차이, 실험 결과, week02 구술 답변 | B1 2·3주차 커리큘럼(김이겸), MuJoCo · ROS 문서 |
| [notes/mujoco-urdf/02-mujoco-advanced.md](notes/mujoco-urdf/02-mujoco-advanced.md) | MJCF 구성 · 구동(계단 응답) · URDF 로딩, week03 구술 답변 | B1 커리큘럼 3주차 10/6 변경본(김이겸), MuJoCo 문서 |
| [notes/usd-isaac/01-usd-basics.md](notes/usd-isaac/01-usd-basics.md) | USD 합성 · USD Physics · Isaac Sim 개념 (3주차 변경 전 선행 학습, 실습 PC 배정 후 사용) | 동, OpenUSD · Isaac Sim 5.1 문서 |

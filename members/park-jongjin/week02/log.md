# week02 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 2 주차 |
| 게이트 | **SRR (×2)** — 통과 기준: 하체 실측 완료 · 목표 사양 확정 · ICD v0 발행 |
| 제출일 | YYYY-MM-DD ← 작성 필요 |
| 저장소 태그 | `week02` |
| 주간 보고서 | reports/ ← 2주차 보고서 제출 후 업로드 |

> **작성 필요:** 비어 있는 칸(실제 수행 · 완료율 · 핵심 수치 · 구술 답변)은 본인이 실제 수행 결과로 채워야 합니다. (추정으로 채우지 말 것 — 구술에서 확인됨)

## 1. 계획 대비 수행

1주차 평가표 「다음 주 목표」 + B1 2주차 커리큘럼(MuJoCo · URDF, GPU 없이) 기준.

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | 실습 PC 배정 시 Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 → create_empty.py · Isaac-Ant-v0 headless 스크린샷 | (PC 배정 여부) | % | `week02/img/` |
| 2 | Cartpole zero_agent · random_agent(`--num_envs 32`) 직접 실행 | | % | |
| 3 | (PC 미배정 시) H1 locomotion env cfg · RewTerm · ObsTerm 정독 → 관측·보상·종료 구조 문서화, 2.3.x ActuatorCfg/SensorCfg 변경 인자 정리 | | % | `notes/…` |
| 4 | Git 저장소 개인 폴더 생성, 노션 정리본을 md로 옮겨 `week02` 태그로 첫 기여 로그 업로드 | 저장소 구조·규칙 수립([README](../../../README.md), [CONTRIBUTING](../../../CONTRIBUTING.md), [템플릿](../../../templates/contribution_log.md), [new_week.sh](../../../tools/new_week.sh)). 노션 페이지 12건(설치 · 데모 · Core Concepts 9항목)을 md 3건으로 이전, 원문 링크·원작성자 표기 | 100 % | [notes/isaac-lab/](../notes/isaac-lab/), 태그 `week02` |
| 5 | (커리큘럼 개별 과제) G1 MJCF 전체 구조 트리 + 우리 로봇(6-DOF 팔 2 + 홀로노믹 베이스) 맞춤 변경 목록 | | % | `week02/g1-structure-tree.md` |
| 6 | (SRR 기여) A팀 요청 항목 초안: CAD 포맷, 질량·관성 제공 형식, 관절 좌표계 규약, 링크 이름 규칙 | | % | `b1/docs/icd/` |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| | | |

## 3. 재현 방법

```bash
# MuJoCo 환경 (Windows, GPU 불필요)
pip install mujoco usd-core yourdfpy
git clone https://github.com/google-deepmind/mujoco_menagerie
python -m mujoco.viewer --mjcf=mujoco_menagerie/unitree_g1/scene.xml
```

## 4. 발견한 문제 · 해결

- **[작성]**

## 5. 막힌 점 · 요청

- (실습 PC 배정 상태, 1주차 질문(v2.1.0 ↔ v2.3.2 차이)에 대한 회신 여부)

## 6. 구술 질문 준비 (커리큘럼 지정)

- Q1. MJCF에서 body와 joint의 부모-자식 관계가 관절 좌표계를 어떻게 정하는가?
  - 답변 요지: **[작성]**
- Q2. inertial 태그를 생략하면 MuJoCo는 어떻게 처리하는가?
  - 답변 요지: **[작성]**

## 7. 다음 주 계획

[week03/log.md](../week03/log.md) 참고.

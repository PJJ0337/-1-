# week03 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 3 주차 |
| 게이트 | 해당 없음 |
| 제출일 | YYYY-MM-DD |
| 저장소 태그 | `week03` (제출 시 부여) |
| 주간 보고서 | reports/ |

## 1. 계획 대비 수행

B1 3주차 커리큘럼(USD 개념 + Isaac Sim 개념 + 변환 파이프라인) 기준.

| No. | 계획 | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | usd-core로 큐브 1개 .usda 생성 → 텍스트로 구조 확인 (Stage · Prim · Attribute · Layer) | | % | `week03/cube.usda` |
| 2 | 2링크 팔을 Xform 계층으로 USD 표현 → RevoluteJoint 2개 + Drive 속성 추가 | | % | `b1/models/usd/` |
| 3 | **(개별)** Isaac Sim 5.1 URDF Importer 문서 기반 「우리 로봇 URDF 임포트 체크리스트」 초안 — 옵션(fix base · merge fixed joints · self-collision 등)별 결과 + 문서 근거 | | % | `b1/docs/procedures/urdf-import-checklist.md` |
| 4 | CAD → URDF → USD(Isaac Sim) / MJCF(MuJoCo) 파이프라인 그림 1장 | | % | `week03/img/pipeline.png` |
| 5 | **(팀 연계)** 함태훈 URDF ↔ USD 대응표 검토 → 체크리스트 반영 | | % | |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| | | [Isaac Sim 5.1 URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/import_urdf.html) |
| | | [USD Physics 스키마](https://openusd.org/release/api/usd_physics_page_front.html) |

## 3. 재현 방법

```bash
```

## 4. 발견한 문제 · 해결

- 

## 5. 막힌 점 · 요청

- 

## 6. 구술 질문 준비 (커리큘럼 지정)

- Q1. Isaac Sim에서 Articulation Root 위치가 왜 중요한가?
  - 답변 요지:
- Q2. merge fixed joints 옵션을 켜면 무엇이 사라지는가?
  - 답변 요지:

## 7. 다음 주 계획

1. (환경 확보 시) 3주차 URDF · USD를 Isaac Sim에 임포트해 Articulation 확인
2. 

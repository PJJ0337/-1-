# week04 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 04 주차 (2026-10-12 ~ 2026-10-16), 마감 10/15(목) |
| 게이트 | 해당 없음 |
| 제출일 | (10/15 예정) |
| 저장소 태그 | `week04` |
| 주간 보고서 | [reports/…docx](../reports/) |

## 1. 계획 대비 수행

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | STEP 링크 분할표 — ICD 이름 {side}_{부위}_{운동} ↔ STEP 부품 ↔ BOM (액추에이터 내부는 한 강체) | (STEP 수령 대기) | 0 % | |
| 2 | (개별) 골격 URDF(link · joint · limit, 베이스 planar) + 구동기 effort · velocity · armature 표(가정값 표 포함), 체크리스트 점검 | 구동기 ICD 요청서 v0 작성 (10/13) | % | b1/docs/icd/actuator-spec-request.md |
| 3 | 함태훈 메시 · inertial 합쳐 v0 URDF(3단 규약) MuJoCo 로드 · 캡처, 구술 대비 | | 0 % | |
| + | (10/8 회신) 체크리스트 v0.3 — balanceinertia 기본 off | 완료 (10/13) | 100 % | b1/docs/procedures/mujoco-urdf-loading-checklist.md |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 (문서 링크 · 실행 로그) |
|---|---|---|
| | | |

## 3. 재현 방법

```bash
# 이번 주 결과를 다시 만들 수 있는 명령 (환경 · 버전 포함)
```

## 4. 발견한 문제 · 해결

- (10/13) 노션 커리큘럼 사본 페이지의 「배포 파일」에 STEP 첨부가 없음 → 원본 페이지에서 수령 필요
- (10/13) `.gitignore`에 `*.step` · `*.stp` · `*.SLDPRT` · `*.zip` · `bom*` 추가 (원본 커밋 금지 규칙)

## 5. 막힌 점 · 요청 (공유한 날짜)

- 

## 6. 구술 질문 준비 (커리큘럼 4주차)

- Q1. joint origin을 CAD 좌표에서 뽑을 때 부모 링크 프레임 기준으로 바꾸는 절차는
  - 답변 요지:
- Q2. BOM의 정격 토크와 피크 토크 중 effort에 무엇을 쓰고, MuJoCo forcerange · ctrlrange와 어떻게 대응되는가
  - 답변 요지: 초안 → [actuator-spec-request.md §3](../../../b1/docs/icd/actuator-spec-request.md)

## 7. 다음 주 계획 (검증 가능한 단위)

1. 
2. 
3. 

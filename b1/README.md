# B1 서브팀 공용 산출물 (모델링 · 시뮬레이션)

개인 학습 결과 중 **서브팀 계획에 바로 쓰이는 것**은 여기로 옮깁니다. 평가표 ① "산출물이 서브팀 계획에 바로 사용됨"(척도 4), ④ "절차서 · BOM · ICD 갱신에 기여"(척도 5)의 근거가 됩니다.

| 폴더 | 내용 | 예정 산출물 |
|---|---|---|
| `models/urdf/` | URDF 모델 | 2링크 팔(2주차) → 우리 로봇 URDF |
| `models/mjcf/` | MuJoCo MJCF 모델 | 2링크 팔 MJCF(2주차) |
| `models/usd/` | Isaac Sim용 USD | 2링크 팔 USD(3주차) |
| `docs/icd/` | 인터페이스 문서(ICD) — A팀 ↔ B1 모델 데이터 규약 | [ICD v0 요청 초안](docs/icd/icd-v0-request-draft.md) (week02) |
| `docs/procedures/` | 절차서 | [URDF 임포트 체크리스트 v0](docs/procedures/urdf-import-checklist.md) (week03), Isaac Sim/Lab 설치 절차 |

파이프라인: CAD(A팀) → URDF → USD(Isaac Sim) / MJCF(MuJoCo)

# week10 기여 로그 — 박종진 (B1) (작성 중)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 10 주차 (2026-11-23 ~ 2026-11-27), 마감 11/26(목) |
| 저장소 태그 | `week10` (제출 시) · week09 = `3c5676c` |
| 함태훈 대응 | taehoonham0118-cell/-1- week10 첫 커밋 `807c788`(11/19) — 데이터 명세 v1.1 |

## 진행 기록

- (11/19) 함태훈 명세 v1.1(`807c788`) 사본 갱신: `datasets/dataset_spec.md` · `b1_dataset.py`. 선택 필드 `action_label`(DART 학습 라벨) · 속성 `dart` · `randomization`, `check()` 선택 필드 검사 · `train_label()`. spec 문자열은 `b1-dataset-1.0` 그대로
  - 새 `check()`로 기존 파일 전부 재검사: 샘플 3/3 · 시연 100/100 · DART 296/296 통과
  - `policies/bc_common.load_dir`가 `b1_dataset.train_label()`로 라벨을 읽게 바꿈 — 바꾸기 전후 특징 · 목표 배열 비트 단위 같음(DART 3 · 시연 2 에피소드 확인)
- (11/19) 팀장 회신(9주차 제출 메시지 답): ① A팀 문안 = 「요구 정격 = 필요 토크 × 1.2 vs 실제 정격」 기준 7 · 8주차 합본으로 팀장 발송, 짐 3 kg 그대로 통과(요구 58.4 < RI85 72.6 · TQ 118.8) — 9주차 3c5676c에서 이미 정정, 용어만 「필요 정격」 → **「요구 정격」**으로 맞춤(torque_budget_b1) ② 어깨 미확정, a2_doc case 계속 ③ 손목은 4-DOF 유지, 예측을 **CDR 설계 제안 근거**로 정리 + 실제 정격 기준 TQ · RI85 여유를 한 표로
  - → `b1/docs/cdr/wrist_2axis_proposal.md`(CDR 자료 1호): 집기 영역 · 깊이 · 도달 상한 + 어깨 요구 정격 · TQ / RI85 여유(pitch · roll) 한 표. `wrist_predict.py`에 roll 추가 · CDR 표 생성. 결과: 손목 달아도 TQ · RI85 둘 다 통과, RI85 pitch 여유 49 → 34 ~ 38 %
- (예정) 랜덤화 환경 재수집 때 속성 `randomization = {"source": "함태훈 randomize.py ed160f2", "f": 1.0, "p": sample_joint(seed, 1.0)}` 기록, `model_hash` = 랜덤화된 모델 지문(함태훈 요청 11/19)

## 계획 (9주차 보고서 4번)

1. BC 강건화: 함태훈 랜덤화 환경(테이블 ±10 mm · 지연 0 ~ 20 ms)에서 시연 · DART 재수집 + 관측에 직전 명령 · 손가락 접촉 → 함태훈 `bc_eval_pjj.py` 조건으로 재평가
2. 끝점 출력도 청킹으로 같은 조건 비교, 상자 · 무거운 물체 실패 분석
3. CDR B1 요약 초안 — 마감 11/26(목)

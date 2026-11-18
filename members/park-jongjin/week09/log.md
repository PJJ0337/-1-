# week09 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 09 주차 (2026-11-16 ~ 2026-11-20), 마감 11/19(목) |
| 커리큘럼 | 노션 「B1 7·8·9주차 학습 커리큘럼」 §3 — 강건성(도메인 랜덤화) + 행동 복제 기초 + CDR · Isaac Lab 이식 준비 (박종진 = BC 학습 · 평가 파이프라인, Isaac Lab 환경 명세 · ArticulationCfg 갱신, 손목 2축 예측 1쪽) |
| 저장소 태그 | `week09` (제출 시) |
| 주간 보고서 | [학부생_주간활동보고서_9주차_박종진_22212289_20261119.docx](../reports/학부생_주간활동보고서_9주차_박종진_22212289_20261119.docx) |
| 함태훈 대응 | taehoonham0118-cell/-1- 최신 = `5bbf722`(11/12, week08 로그 정정). **week09(randomize.py · 민감도 · EventTerm 매핑) 아직 없음(11/18 확인)** → BC 「랜덤화 있음」은 임시판 `policies/rand_dyn.py`로 먼저, 받으면 교체(§5) |

## 1. 계획 대비 수행

| No. | 계획 (8주차 보고서 · 커리큘럼) | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 1 | BC 학습 · 평가 파이프라인(HDF5 로더 · 정규화 · 학습/검증 분리 · MLP CPU) → 롤아웃 성공률: 전문가 vs BC, 랜덤화 없음/있음, BC GIF | 로더(명세 v1.0 → 로봇 좌표 특징 39) · PyTorch CPU MLP · 에피소드 단위 검증 분리 · 처음 보는 seed 50개 롤아웃 + 실패 분류. **전문가 98 % / BC 0 % → DART 노이즈 시연 + 액션 청킹 25로 36 %(랜덤화 있음 24 %)**. GIF(seed 2004 원기둥 성공) | 100 % (랜덤화는 임시판) | policies/*, results/eval.md, img/bc_eval.png · bc_joint_dart_c25_seed2004.gif |
| 2 | BC 출력: 관절 목표 vs 끝점 목표 비교 (함태훈 week08 action_format 후속) | 같은 설정(시연 100개) 닫힌 루프 둘 다 0 %, 검증 open-loop 끝점 목표 오차 관절 0.24 · 끝점 0.22 mm(평균) — 차이 없음, 끝점은 롤아웃마다 IK 1회(잔차 평균 0.09 mm) 추가. **실패 모양이 다름**(§2) | 100 % | results/offline_compare.md, eval.md |
| 3 | Isaac Lab 환경 명세 `isaaclab_env_spec.md` + ArticulationCfg 갱신, 함태훈 Event 매핑 통합 | 명세 v0.1(Scene · Obs 39 · Action 3종 · Event · Termination · Reward 초안 · 사용자 정의 함수 목록 · 첫날 확인 순서). **손가락 URDF 사본 + 그리퍼 액추에이터 · 테이블 · 물체 · 접촉 센서 cfg 생성**(원본 URDF · MJCF 그대로). 함태훈 Event 열은 비워 둠 | 90 % (함태훈 매핑 대기) | b1/docs/isaaclab_env_spec.md, b1/models/v1/scripts/export_isaac_pick.py → urdf/b1_v1_isaac_pick.urdf · isaac/b1_pick_cfg.py |
| 4 | 손목 2축 예측 1쪽 | 모델 변경 없이 플랜지 자세 + 손목 수식: 수직 하강 면적 260 → 1,744 cm²(6.7배), 옆면 깊이 0.302 → 0.413 m, 팔만 IK 0 → 6.7 %, 전신 상한 78.7 → 100 %, 지금 실패 2건 해소 예상 · 대신 PDR 52 여유 6.8 → −0.8 % | 100 % | week09/wrist_prediction.md, b1/models/v1/scripts/wrist_predict.py, results/wrist_predict.md, img/wrist_predict.png |
| 5 | 대기 항목: 함태훈 명세 수정본 사본, 팀장 · 구술 피드백 | dataset_spec.md 사본을 함태훈 716ec3b(시간 항목 0.5 ms · 40 스텝)로 갱신(필드 변경 없음 → 데이터 재생성 불필요). 팀장 · 평가 회신은 미수령 | 50 % (외부 대기) | datasets/dataset_spec.md |

## 2. 핵심 수치 · 근거 (구술 대비)

평가 = seed 2000 ~ 2049(학습 1000 ~ 1099 · DART 3000 ~ 3299와 분리), 공통 판정(목표 원 40 mm · 테이블 위 · 기울기 < 15° · 놓음 · 테이블 충돌 없음), BC 34 s 한도

| 정책 | 랜덤화 없음 | 랜덤화 있음 | 주된 실패 | 근거 |
|---|---|---|---|---|
| 전문가(상태기계) | **49/50 (98 %)** | **49/50 (98 %)** | 계획 IK 실패 1(seed 2002) | results/eval.md |
| BC 관절 · 시연 100 | 0/50 | 0/50 | 테이블 충돌 35 · 못 집음 15 | 〃 |
| BC 끝점 · 시연 100 | 0/50 | 0/50 | 테이블 충돌 31 · 못 집음 14 · **수치 발산 5**(IK 잔차 에피소드 최대 중앙값 75 mm — 예측 끝점 + 축 5조건이 4-DOF 팔 곡면 밖) | 〃 |
| BC 관절 · + DART 148 | 0/50 | 0/50 | 못 집음 46(경유점에서 멈춤) | 〃 |
| **BC 관절 · + DART · 청킹 25** | **18/50 (36 %, CI 24 ~ 50)** | **12/50 (24 %)** | 들고 시간 초과 · 테이블 충돌 · 수치 발산 · 낙하 | 〃, img/bc_eval.png |
| BC 관절 · + DART · 청킹 50 | 11/50 (22 %) | 13/50 (26 %) | 못 집음 14 · 테이블 9 | 〃 |

- 검증 MSE(정규화) 0.0019 · 검증 시연 끝점 목표 오차 평균 0.24 mm인데 롤아웃 0 % = 공변량 이동(open-loop로는 거의 완벽) — offline_compare.md
- 랜덤화 하락폭(청킹 25): 36 → 24 % (−12 %p), 전문가는 98 → 98 %(닫힌 루프 피드백 + 계획 단계가 동역학과 무관). 청킹 50은 22 → 26 %로 표본 50개 CI 안 차이
- 성공 에피소드: 원기둥 11/23 > 상자 7/27(청킹 25, 랜덤화 없음), 성공 물체 질량 중앙값 0.88 kg vs 실패 1.63 kg(원기둥) → 무거운 물체 · 상자(방향 맞춤)에서 약함. 성공 시 목표 오차 중앙값 3.6 mm(전문가 4.7 mm), 놓은 뒤 0.5 s 유지에서 종료 → 평균 15.8 s(전문가는 복귀까지 24.6 s)
- DART 수집: 300 seed → 성공 148(19.7만 프레임), 실패는 노이즈 때문에 허용 오차에 못 들어온 approach_timeout 89 · align_fail 45 — results/collect_dart.md
- 학습 시간(2코어 CPU): 시연 100 → 33 s, + DART → 84 ~ 183 s, 청킹 25 → 133 s

가정값: 그리퍼(S15) · 접촉(S16) · 집기 환경(S17) · 어깨 a2_doc. 랜덤화 범위 = rand_dyn.py(가정값 표 불확실도) — 함태훈 범위로 교체 예정

### 1차 탐색(평가 seed 2000 ~ 2007/2009, 8 ~ 10개 — 방향 잡기용, 체크포인트는 커밋 안 함)

| 설정 | 성공 | 실패 모양 |
|---|---|---|
| 시연 100 · 직전 행동 입력 | 0/8 | 테이블 충돌 8 (경유점 전에 베이스가 앞으로) |
| 시연 100 · 경과 시간 입력 | 0/8 | 테이블 4 · 못 집음 4 |
| + DART · 직전 행동 | 0/10 | 테이블 7 — 파지까지 가지만 베이스가 계속 회전(copycat) |
| + DART · 경과 시간 | 0/10 | 못 집음 8 |
| + DART · 직전 행동 + 입력 노이즈 0.5 | 0/10 | 못 집음 10(다시 멈춤) |
| + DART · 청킹 10 (0.2 s) | 2/10 | 못 집음 5 — 0.2 s 앞으로는 멈춤을 못 풂 |
| + DART · 청킹 25 · 50 | 2/10 · 2/10 | → 50개 본 평가 |

## 3. 재현 방법

```bash
pip install mujoco==3.14.0 numpy scipy h5py imageio matplotlib torch
python scenarios/collect.py --n 100 --start 1000                     # 8주차 시연 → datasets/raw (커밋 안 함)
python policies/collect_dart.py --n 300 --start 3000                 # DART → datasets/raw_dart (커밋 안 함)
python policies/train_bc.py --out joint --data datasets/raw,datasets/raw_dart --chunk 25 --tag joint_dart_c25
python policies/eval_bc.py --policies expert,joint,ee,joint_dart,joint_dart_c25,joint_dart_c50 --conds nom,rand --n 50
python policies/plot_week09.py
python b1/models/v1/scripts/export_isaac_pick.py                     # 손가락 URDF 사본 · 집기 scene cfg
python b1/models/v1/scripts/wrist_predict.py                         # ~3 분
```
- 클라우드 리눅스(mujoco 3.14.0)에서 시연 100개 재수집 = 100/100 · 123,040 스텝(노트북 8주차 값과 같음), 모델 지문은 기기 · 버전이라 다름(eb2b9500cc44 vs 4d070b18a640)

## 4. 발견한 문제 · 해결

- (11/18) 첫 BC 0/8: open-loop는 맞는데 베이스가 경유점에 0.3 rad 못 미친 채 다음 동작 → 테이블 충돌. 시연에는 「경로에서 벗어난 상태」가 없음 → **DART**(실제 명령에만 OU 노이즈, 라벨은 깨끗한 명령). 처음 σ 팔 0.03 rad는 6/6 실패(끝점 ~15 mm > 정렬 허용 6 mm) → 0.01 rad + 정밀 단계 30 %
- (11/18) DART 후 테이블 충돌 0에 가깝지만 **경유점에서 멈춤**: 전문가 구간이 5차 궤적이라 정지 상태의 라벨 ≈ 0(숨은 변수 = 궤적 시간). 직전 행동 입력 → 멈춤은 풀리지만 copycat, 경과 시간 입력 → 안 풀림 → **액션 청킹**(0.5 s 앞까지 예측 + 시간 앙상블)으로 해결
- (11/18) DART 수집 중 수치 발산 에피소드가 명세 check(시간 간격)에 걸려 수집이 멈춤(MuJoCo가 발산 때 시간을 리셋) → 그 에피소드는 저장하지 않고 기록만(4/300)
- (11/18) 학습 두 개 동시 실행 시 2코어에서 스레드 경합으로 10배 느려짐 → 순서대로 실행
- (11/18) torch CPU 휠(download.pytorch.org)이 클라우드 프록시에서 막혀 PyPI 기본 휠로 설치(동작 동일, 노트북은 CPU 휠 권장)

## 5. 막힌 점 · 요청 (공유한 날짜)

- (함태훈, 11/18) week09 randomize.py · 민감도 Top 3 · EventTerm 매핑 push 요청 → 받으면 ① `rand_dyn.apply`를 교체해 BC 「랜덤화 있음」 재평가 ② isaaclab_env_spec.md §6 「함태훈」 열 통합(v0.2). 내 쪽 BC 실패 사례(eval.json rows: seed별 실패 · 랜덤 표본)를 민감 파라미터와 연결해 달라는 요청도 같이
- (함태훈) DART 시연은 명세 v1.0에 확장 필드 `action_label`(전문가 명령)을 하나 더 씀 — `action` = 실제로 넣은 명령이라 재생 규칙(§4)은 그대로. 명세에 「선택 필드」로 넣을지 확인 요청
- (팀장) A팀 회신안(7 · 8주차) 검토 · 전달, PDR 52 판단(짐 2 kg / 최악 자세 금지), 어깨 구동기 확정 — 대기. **손목 2축 예측은 토크 쪽에 불리**(PDR 52 여유 −0.8 ~ −4 %) → 손목 논의는 어깨 확정과 같이
- (평가) 5 ~ 8주차 평가표 · 구술 피드백, 실습 PC 배정(김이겸) — 대기

## 6. 구술 질문 준비 (커리큘럼 9주차)

- Q1 BC < 전문가 이유(공변량 이동)와 줄이는 방법 / Q2 MuJoCo 시나리오 요소 ↔ Isaac Lab Manager-Based term → [notes/isaac-lab/04-bc-isaaclab-mapping.md](../notes/isaac-lab/04-bc-isaaclab-mapping.md)

## 7. 다음 주 계획 (10주차, 검증 가능한 단위)

1. 함태훈 randomize.py 반영: BC(청킹 25) 랜덤화 있음 재평가 + 실패 사례 ↔ 민감 파라미터 표, isaaclab_env_spec v0.2
2. BC 개선 1단계: 상자 · 무거운 물체 실패 분석, 끝점 출력에도 청킹 적용해 관절과 같은 조건 비교, 시연 수 100 → 300 효과(학습 곡선)
3. CDR B1 요약 초안(모델 · 가정값 · 검증 · 토크 · 작업공간 · 시나리오 · 데이터 · 강건성 · BC) — 12주차 CDR 자료

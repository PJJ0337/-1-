# 시연 데이터 수집 · 재생 검증 (자동 생성 — collect_report.py)

- 전문가 정책(상태기계 + 전신 IK) 자동 생성 **100개**(seed 1000 ~ 1099, 평가용 seed 0 ~ 19와 분리): 성공 **100/100**, 실패 {}
- 물체 상자 41 · 원기둥 59, 질량 0.2 ~ 3 kg 로그 균등. 총 123,040 스텝(50 Hz, 에피소드 평균 1230 스텝 ≈ 24.6 s), 원본 50.5 MB(gzip) — 커밋 안 함(`datasets/raw/`)
- 수집 159.7 s (CPU 1코어, dt 0.5 ms), 모델 지문 `4d070b18a640`, 형식 = 함태훈 v1.0 `b1-dataset-1.0`(datasets/dataset_spec.md)

## 재생(replay) 검증

- 명세 검사(함태훈 `b1_dataset.check`): 100/100 통과
- 저장된 action만 다시 넣어 시뮬(같은 seed로 초기화) → **100개 전부 qpos 최대 차 0.0e+00**(비트 단위 일치), 성공 판정 일치 100/100, 모델 지문 일치 100/100, 101.3 s
- 커밋한 샘플 3개도 같은 검증: 최대 차 0.0e+00, 지문 일치 3/3
- 결정성 조건: 같은 모델(지문) · 같은 MuJoCo 버전(3.15.0) · 단일 스레드 · 같은 초기 상태(seed → `sample_task`)
- week08(dt 1 ms, da55741)에서는 99/100 — seed 1080(원기둥 2.98 kg) 들기 순간 수치 발산(sim_unstable), 재생에서도 같은 시각(11.483 s)에 똑같이 발산. 함태훈 원인 분석(물체 각속도 0.1 → 17 rad/s 지수 증가 = 수치 진동, solref · solimp · impratio · condim으로는 안 풀리고 dt 0.5 ms만 해결) → dt 0.5 ms로 재수집

## 실패 에피소드

| seed | 물체 | 질량 kg | 실패 | 스텝 |
|---|---|---|---|---|
| — | | | 없음 | |

## 커밋한 샘플 (`datasets/samples/`)

| 파일 | 내용 |
|---|---|
| ep_01000.hdf5 | 상자 0.84 kg, 카메라 `overview` 96 × 96 포함 (~7 MB) |
| ep_01001.hdf5 · ep_01002.hdf5 | 원기둥, 카메라 없음 (~0.5 MB) |

형식: [../../datasets/dataset_spec.md](../../datasets/dataset_spec.md) v1.0 (함태훈, 초안 [dataset_spec_draft.md](../../datasets/dataset_spec_draft.md)의 필드 그대로 + 로봇 좌표 파생 4개 · 속성 5개)

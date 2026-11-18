# policies/ — 행동 복제(BC) 학습 · 평가 (week09, 박종진)

8주차 시연 데이터(`b1-dataset-1.0`)로 상태 기반 MLP 정책을 CPU에서 학습하고, 처음 보는 seed에서 전문가와 같은 판정으로 롤아웃 평가한다. 수집 → 학습 → 평가가 끝까지 도는 파이프라인 확인이 목적(커리큘럼 9주차).

| 파일 | 내용 |
|---|---|
| `bc_common.py` | 관측 특징 39(로봇 좌표) · 행동 목표(관절 8 / 끝점 10 / 청킹 K×8), HDF5 로더, 롤아웃용 `live_features`, 팔 FK · IK(`Kin`) |
| `train_bc.py` | PyTorch CPU 학습: MLP 3 × 256, 정규화, 에피소드 단위 검증 분리(seed 1090 ~ 1099), 옵션 `--data a,b`(DART 추가) · `--chunk K` · `--hist prev,time` · `--noise` |
| `collect_dart.py` | DART 노이즈 시연: 실제 명령 = 전문가 + OU 노이즈, 라벨 = 전문가 명령(`action_label` 확장 필드) → `datasets/raw_dart/`(커밋 안 함) |
| `rand_dyn.py` | 동역학 랜덤화 임시판 7종(범위 근거 = 가정값 표) — **함태훈 randomize.py로 교체 예정** |
| `eval_bc.py` | 롤아웃 평가: 전문가 · BC × 랜덤화 없음/있음, seed 2000 ~ 2049, 공통 판정 · 실패 분류, 청킹 시간 앙상블 |
| `offline_compare.py` | 관절 vs 끝점 출력: 검증 시연 open-loop 오차를 끝점 공간 같은 단위로 |
| `plot_week09.py` · `collect_dart_report.py` · `render_bc_gif.py` | 그림 · 표 · GIF |
| `ckpt/bc_*.pt` | 체크포인트(정규화 포함, 0.6 ~ 1.0 MB): joint · ee · joint_dart · joint_dart_c25 · joint_dart_c50 |
| `results/` | `eval.{json,md}` · `train_*.json` · `offline_compare.*` · `collect_dart.*` |
| `img/` | `bc_eval.png`, BC 롤아웃 GIF · 장면 띠 |

## 실행 (노트북 · CPU, 저장소 루트에서)

```bash
pip install mujoco==3.14.0 numpy h5py imageio matplotlib torch   # torch CPU 판이면 충분
python scenarios/collect.py --n 100 --start 1000                  # 8주차 시연(datasets/raw, ~3 분)
python policies/collect_dart.py --n 300 --start 3000              # DART 시연(datasets/raw_dart, ~8 분)
python policies/collect_dart_report.py 300
python policies/train_bc.py --out joint                            # ~1 분
python policies/train_bc.py --out ee
python policies/train_bc.py --out joint --data datasets/raw,datasets/raw_dart --tag joint_dart
python policies/train_bc.py --out joint --data datasets/raw,datasets/raw_dart --chunk 25 --tag joint_dart_c25   # ~3 분
python policies/train_bc.py --out joint --data datasets/raw,datasets/raw_dart --chunk 50 --tag joint_dart_c50
python policies/eval_bc.py --policies expert,joint,ee,joint_dart,joint_dart_c25,joint_dart_c50 --conds nom,rand --n 50   # ~30 분(2 프로세스)
python policies/offline_compare.py joint ee joint_dart
python policies/plot_week09.py
MUJOCO_GL=egl python policies/render_bc_gif.py joint_dart_c25 2004   # Windows는 MUJOCO_GL 없이
```

## 결과 요약 → [results/eval.md](results/eval.md)

- 전문가 98 % · BC(시연 100개 그대로) 0 % — 검증 손실은 아주 작음(끝점 목표 오차 평균 0.24 mm)인데 닫힌 루프에서 실패 = **공변량 이동**
- DART(노이즈 시연 148개 추가) → 테이블 충돌은 사라지지만 구간 사이 「멈춤」에서 못 벗어남(0 %)
- DART + **액션 청킹 25(0.5 s)** → 36 % (랜덤화 있음 24 %)
- 학습 · 평가 seed는 모두 분리(학습 1000 ~ 1099 · DART 3000 ~ 3299 · 평가 2000 ~ 2049)

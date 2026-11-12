# scenarios/ — 접촉 집기 시나리오 · 시연 데이터 (week08, 박종진)

B1 v1 모델(`b1/models/v1`, 생성된 MJCF는 손대지 않음)에 테이블 · 물체 2종 · 임시 평행 2지 그리퍼를 **메모리에서** 덧붙여, 물체를 실제로 잡아 옮기는 시나리오와 시연 데이터 수집 파이프라인.

| 파일 | 내용 |
|---|---|
| `pick_env.py` | scene 생성(`build`), 가정값 dict(`GRIPPER` · `CONTACT` · `TABLE` · `OBJ` · `SPAWN` · `GOAL` · `OPTION`), seed → 과제(`sample_task`), 초기화(`reset`), 접촉 분류 |
| `expert.py` | 상태기계 + 전문가 정책(전신 IK `wb_ik` + resolved-rate), 성공 판정 · 실패 분류, `apply_action`(action 8 → ctrl 13), `run_episode` |
| `gripper_slip.py` | 그리퍼 파지 유지 점검: 마찰 × 파지력 × 질량 54조합 + 마찰 원뿔 · condim 비교 → `results/gripper_slip.md` |
| `run_pick20.py` | 무작위 20회 성공률 · 실패 분류 + 깊이별 계획 성공(도달 한계) → `results/pick20.md` |
| `collect.py` | 시연 데이터 수집(HDF5) · 재생 검증 → `results/collect.md`, 형식 = [`../datasets/dataset_spec.md`](../datasets/dataset_spec.md) v1.0(함태훈) — 초안으로 쓴 뒤 `datasets/b1_dataset.py` upgrade · check |
| `render_pick_gif.py` | 집기 GIF → `img/pick_seed6.gif` |

## 실행 (노트북 · CPU)

```bash
pip install mujoco==3.15.0 numpy scipy h5py pillow
python scenarios/gripper_slip.py                       # ~5 s
python scenarios/run_pick20.py                         # ~90 s (20회 + 깊이 점검)
python scenarios/collect.py --n 100 --start 1000       # ~3.5 분 → datasets/raw/ (커밋 안 함)
python scenarios/collect.py --replay datasets/raw      # ~2 분, 재생 일치 + v1.0 check
python scenarios/collect.py --n 1 --start 1000 --cam --out datasets/samples   # 샘플(카메라 포함)
python scenarios/collect.py --n 2 --start 1001 --out datasets/samples
python scenarios/collect_report.py                     # → results/collect.md
MUJOCO_GL=egl python scenarios/render_pick_gif.py 6    # Windows는 MUJOCO_GL 없이
```

`results/down_poses.npz`(수직 하강 팔 자세 라이브러리)는 첫 실행 때 만들어 캐시(~10 s).

## 시나리오 요약

- 테이블 윗면 0.70 m · 앞 가장자리 world x 0.45 m. 로봇은 원점(정면 +x)에서 시작 → **옆면 접근(베이스 yaw ≈ −90°) + 수직 하강**(7주차 workspace_wb ④, 함태훈 11/05 합의)
- 단계: APPROACH → ALIGN → DESCEND → GRASP → LIFT → MOVE → PLACE → RELEASE → RETREAT → RETURN, 전환은 측정값(끝점 오차 · 접촉 · 손가락 속도 · 물체 높이)으로
- 성공 = 물체가 목표 원(반지름 40 mm) 안 · 테이블 위(높이 ±10 mm) · 기울기 < 15° · 실패 이벤트 없음
- 그리퍼 · 접촉 · 테이블 · 물체 값은 전부 **가정값**(그리퍼 사양 미수령, 함태훈 week08 스윕에서 그대로 채택). 어깨 구동기 = a2_doc case
- 시뮬 dt **0.5 ms**(제어 50 Hz = 40 스텝): 1 ms에서는 3 kg 원기둥 들기 순간 수치 진동 발산(seed 1080) — 함태훈 contact_sweep에서 dt만 해결(week09 반영, 계산 1.9배)

# B1 시연 데이터 형식 — 초안 v0.1 (week08, 박종진) — **대체됨 → [dataset_spec.md](dataset_spec.md) v1.0 (함태훈 7f434ae)**

> **상태**: 박종진 초안. 커리큘럼상 데이터 명세 `docs/dataset_spec.md`는 함태훈 담당 — 11/12 기준 함태훈 저장소에 아직 없어 수집 스크립트가 따를 형식을 먼저 정함. 함태훈 명세가 오면 필드 이름 · 단위를 그쪽에 맞추고 `scenarios/collect.py`의 `save()`만 고친다(재수집 ~2 분).
>
> 생성: `scenarios/collect.py` (spec 문자열 `b1-w08-draft-0.1`), 검증: `python scenarios/collect.py --replay <폴더>`

## 1. 단위 · 파일

- 에피소드 1개 = HDF5 파일 1개 `ep_<seed 5자리>.hdf5`. 대용량 원본(`datasets/raw/`, 100개 ≈ 45 MB)은 커밋하지 않음(.gitignore), **샘플 3개만** `datasets/samples/`(카메라 포함)
- 시간 축 길이 T = 제어 주기(50 Hz) 수. 관측 `obs/*[t]` = t번째 명령을 내기 **직전** 상태, `action[t]` = 그 뒤 0.02 s(시뮬 20 스텝, dt 1 ms) 동안 유지한 명령
- 좌표 = world(MuJoCo), 길이 m · 각도 rad · 쿼터니언 [w, x, y, z]

## 2. 필드

| 경로 | 모양 · 형 | 내용 |
|---|---|---|
| `obs/qpos` | (T, 27) f64 | 전체 qpos: 베이스 TX · TY · RZ(3) · 왼팔 4 · 왼손가락 2 · 오른팔 4 · 상자 free(7) · 원기둥 free(7). 재생 비교 기준이라 f64 |
| `obs/qvel` | (T, 25) f32 | 전체 qvel (같은 순서, free joint는 6) |
| `obs/ee_pos`, `obs/ee_quat` | (T, 3) · (T, 4) f32 | 왼손 끝점 site `left_ee`(손바닥 아래면 중심). 파지 중심 = ee − 0.045 ẑ_ee |
| `obs/obj_pos`, `obs/obj_quat` | (T, 3) · (T, 4) f32 | 집는 물체 몸체 위치 · 자세 |
| `obs/gripper_width` | (T,) f32 | 손가락 안쪽 면 사이 거리 [m] (0 ~ 0.11) |
| `obs/stage` | (T,) i8 | 상태기계 단계 번호(속성 `stage_names`) — 학습 입력이 아니라 분석 · 분할용 |
| `obs/images/overview` | (T, 96, 96, 3) u8 | (선택, `--cam`) 고정 카메라 `overview` |
| `action` | (T, 8) f64 | `[base_vx, base_vy, base_wz, left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow, gripper_opening]` — 베이스 = world 속도 명령(m/s, rad/s), 팔 = 관절 위치 목표(rad), 그리퍼 = 벌림 목표(손가락 1개 m, 0 ~ 0.055). 오른팔은 영점 고정이라 제외 |
| `ctrl` | (T, 13) f64 | 실제 d.ctrl 전체(액추에이터 순서) — action → ctrl 변환(`expert.apply_action`) 확인용 |
| `time` | (T,) f64 | 시뮬 시간 |
| `final_qpos` | (27,) f64 | 마지막 명령 뒤 상태(재생 끝점 비교) |

## 3. 속성 (meta, `f.attrs`)

`spec`, `seed`(→ `pick_env.sample_task(seed)`가 물체 종류 · 위치 · yaw · 질량을 다시 만듦), `kind`, `mass`, `obj_xy`, `obj_yaw`, `success`, `fail`, `fail_stage`, `goal_err_mm`, `t_total`, `control_hz`(50), `sim_dt`(0.001), `model_hash`(컴파일 모델 sha256 앞 12자리 — 다르면 재생 보장 안 됨), `action_names`, `stage_names`, `params`(그리퍼 · 접촉 · 테이블 · 목표 · 물체 · option · 무작위 범위 JSON), `events`(단계 전환 시각)

## 4. 재생(replay) 규칙

1. `m = pick_env.build(cam=True)`, `I = reset(m, d, sample_task(seed))` — 같은 seed = 같은 초기 상태
2. t = 0 … T−1: `apply_action(m, d, action[t])` → `mj_step` 20번
3. 매 t에서 `d.qpos`와 `obs/qpos[t]`, 끝에서 `final_qpos` 비교 → 최대 절대 차(결정성이면 0)

## 5. 정해야 할 것 (함태훈과)

- action을 「관절 목표」로 둘지 「끝점 목표(위치 + 자세)」로 둘지 — 지금은 관절 목표(재생이 정확, 로봇에 그대로). 끝점 목표는 `obs/ee_*` 차분으로 만들 수 있음 (함태훈 구술 ② 주제)
- 베이스 action = world 속도 → 로봇 기준 속도(R(yaw)ᵀ)로 바꿀지 (Isaac Lab 이식 시 로봇 기준이 자연스러움)
- LeRobot 형식 변환(`observation.state` · `action` · `episode_index` · `frame_index` · `timestamp`) — 9주차 BC 로더에서

# IK 함수 인터페이스 (week06 v1 — 구현 완료 10/27, 박종진 → 함태훈 집기 시나리오용. 초안 10/18)

커리큘럼 6주차 「팀 연계」: 박종진 IK 모듈 → 함태훈 집기 시나리오. 구현 = `b1/models/v1/scripts/ik.py`, 결과 = `results/ik_results.md`.

```python
import sys; sys.path.insert(0, "b1/models/v1/scripts")
from ik import solve_ik, IKResult, approach_quat, load_b1   # load_b1() = (model, data) — 한글 경로 대응 로더

res: IKResult = solve_ik(
    model, data,                 # mujoco.MjModel / MjData (b1.xml · scene.xml). data는 내부에서 복사본으로 계산 → 호출 쪽 상태 안 바뀜
    side,                        # "left" | "right"
    target_pos,                  # (3,) [m] — frame 기준 목표 위치 (ee site = 그리퍼 끝)
    target_quat=None,            # (4,) [w, x, y, z] MuJoCo 순서, None이면 위치만
    *,
    frame="base",                # "base" = base_link 기준(베이스 이동과 무관) | "world"
    q_init=None,                 # (4,) [rad] 시작 관절각 (None이면 현재 data.qpos의 해당 팔)
    rot_mode="axis",             # target_quat 있을 때: "axis" = 그리퍼 축(ee site z축)만 맞춤(4-DOF라 기본) | "full" = 자세 전체(가중 최소자승)
    rot_weight=0.3,              # 자세 오차 가중 [m/rad]
    damping=0.05,                # DLS λ
    pos_tol=1e-3, rot_tol=1e-2,  # [m], [rad]
    max_iter=200,
    step_max=0.3,                # [rad] 반복당 ‖dq‖ 상한 (특이점 근처 튐 방지)   ← v1 추가
    restarts=4, seed=0,          # 수렴 실패 시 관절 범위 안 무작위 시작점 재시도 횟수  ← v1 추가
    return_history=False,        # True면 (IKResult, [(pos_err, rot_err, ‖dq‖), …]) 반환    ← v1 추가
)
# IKResult (dataclass)
#   q: np.ndarray (4,)      [pitch, roll, yaw, elbow] rad — 관절 범위 안으로 클리핑된 값 (그대로 position ctrl에 넣으면 됨)
#   success: bool           pos_err ≤ pos_tol (그리고 자세 지정 시 rot_err ≤ rot_tol)
#   pos_err: float [m], rot_err: float [rad], iters: int
#   status: str             "converged" | "orientation_limit"(v1 추가: 위치는 맞췄고 그리퍼 축만 최근접) | "max_iter" | "unreachable"(작업공간 밖 — 최근접 해 반환) | "joint_limit"
#   restarts_used: int      몇 번째 시작점에서 나온 해인지 (0 = q_init)
```

- 관절 순서 · 이름 = ICD `{side}_{shoulder_pitch, shoulder_roll, shoulder_yaw, elbow}_joint`, ctrl 이름 = `{side}_{...}` (액추에이터)
- 4-DOF라 위치 3 + 자세 3을 다 맞출 수 없음 → 기본은 위치 + 그리퍼 축 방향(5개 조건 중 4자유도, 최소자승). 구술 Q2(자코비안 3 × 4 vs 6 × 4) 연결
- 작업공간 밖 목표: 실패를 숨기지 않고 `success=False`, `status="unreachable"`, 최근접 q 반환 → 집기 시나리오에서 판단
- 토크 예산표용: 작업공간 분석에서 「도달 가능 자세 중 관절별 정적 토크 최대 자세」를 `results/workspace_max_torque.json`(관절별 q · ee 위치 · τ, 페이로드 0 / 0.5 / 1.0 / 3 kg)으로 공유

## v1 변경 · 사용 팁 (10/27)

- **그리퍼 축 지정**: `approach_quat([1, 0, 0])` = 정면으로 접근, `approach_quat([0, 0, -1])` = 위에서 아래로. ee site의 −z가 그 방향을 향함(axis 모드는 z축만 사용)
- **4-DOF 한계**: 손목이 없어 「위치 + 그리퍼 축」(5조건)을 동시에 만족하는 점은 2차원 곡면뿐 → 대부분의 점에서 `orientation_limit`. 이때 위치는 1 mm 이내, 축은 영공간(1차원)에서 최소(SLSQP). 예: 정면 접근은 팔을 다 편 x ≈ 0.595 m에서만, 수직 하강 접근은 roll = yaw = 0 평면에서 `ik_results.json` `pick_point` 같은 점
- **집기 시나리오 제안**: 목표 위치는 위치만 IK(수렴률 99.6 %), 하강 · 상승은 같은 해 근처 q_init으로 연속 호출(해 가지 유지). 그리퍼 방향이 필요하면 `ik_results.json`의 `pick_point`(수직 접근 실현점) 기준으로 목표를 잡기
- **작업공간 밖**: 상판 윗면(z 0.40 m) ~ z 0.606 m는 팔이 닿지 않음 → T10 `unreachable`. 바닥 · 상판 위 물체 집기는 현재 기구로 불가(A팀 피드백 항목)
- 토크 예산표 최악 조건: `results/workspace_max_torque.json` (관절별 q · ee 위치 · τ, 페이로드 0 / 0.5 / 1.0 / 3 kg) — 완료

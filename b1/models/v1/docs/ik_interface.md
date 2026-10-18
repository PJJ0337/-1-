# IK 함수 인터페이스 (week06 초안 — 박종진 · 함태훈 합의용, 2026-10-18)

커리큘럼 6주차 「팀 연계」: 박종진 IK 모듈 → 함태훈 집기 시나리오. 구현은 week06 `b1/models/v1/scripts/ik.py`.

```python
from ik import solve_ik, IKResult

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
)
# IKResult (dataclass)
#   q: np.ndarray (4,)      [pitch, roll, yaw, elbow] rad — 관절 범위 안으로 클리핑된 값 (그대로 position ctrl에 넣으면 됨)
#   success: bool           pos_err ≤ pos_tol (그리고 자세 지정 시 rot_err ≤ rot_tol)
#   pos_err: float [m], rot_err: float [rad], iters: int
#   status: str             "converged" | "max_iter" | "unreachable"(작업공간 밖 — 최근접 해 반환) | "joint_limit"
```

- 관절 순서 · 이름 = ICD `{side}_{shoulder_pitch, shoulder_roll, shoulder_yaw, elbow}_joint`, ctrl 이름 = `{side}_{...}` (액추에이터)
- 4-DOF라 위치 3 + 자세 3을 다 맞출 수 없음 → 기본은 위치 + 그리퍼 축 방향(5개 조건 중 4자유도, 최소자승). 구술 Q2(자코비안 3 × 4 vs 6 × 4) 연결
- 작업공간 밖 목표: 실패를 숨기지 않고 `success=False`, `status="unreachable"`, 최근접 q 반환 → 집기 시나리오에서 판단
- 토크 예산표용: 작업공간 분석에서 「도달 가능 자세 중 관절별 정적 토크 최대 자세」를 `results/workspace_max_torque.json`(관절별 q · ee 위치 · τ, 페이로드 0 / 0.5 / 1.0 / 3 kg)으로 공유

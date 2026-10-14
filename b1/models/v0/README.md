# b1_humanoid_v0 — 우리 로봇 첫 시뮬레이션 모델 (week04)

STEP(`양팔로봇 최종버전.stp` · `BASE 3.STEP`) → 링크 분할 → 메시 · 관성 → URDF → MuJoCo 3단 MJCF.
**가정값이 많음** — 반드시 [../assumptions-v0.md](../assumptions-v0.md) 와 같이 볼 것.

```
v0/
├── urdf/b1_humanoid_skeleton.urdf   골격: link · joint · limit + CAD 관성, 형상 없음
├── urdf/b1_humanoid_v0.urdf         v0: 골격 + visual(단순화 STL) · collision(convex hull STL)
├── meshes/*.stl (18개)              링크 프레임 · m 단위, right = left 거울, mesh_info.json
└── mjcf/robot.xml                   ← v0.urdf 를 MuJoCo로 읽어 mj_saveLastXML 한 변환본 (수정 금지)
    mjcf/robot_actuated.xml          ← include robot.xml + position 8 · velocity 3 액추에이터 · 센서
    mjcf/scene.xml                   ← include robot_actuated.xml + 바닥 · 조명 · option
```

| 구성 | 값 |
|---|---|
| 링크 / 관절 | odom →(planar)→ base_link →(fixed)→ torso_link → {left,right}_{shoulder_pitch, shoulder_roll, shoulder_yaw, elbow}_link → {side}_tool0 |
| 영점 | 팔 내림 + 팔꿈치 전방 굽힘(팀장 10/14) — `shoulder_yaw` origin rpy ∓π/2, elbow + = 전방 굽힘 |
| 자유도 | nq = nv = 11 (베이스 평면 3 + 팔 4 × 2), nu = 11 |
| 좌표 | base_link = REP 103(x 전방 · y 좌 · z 상), 원점 = 바닥면 중심. CAD 좌표는 `cad_frame`(torso_link 아래 고정 프레임) 하나로만 연결 — CAD 점 p는 cad_frame 기준 p/1000. 어깨 J1축 높이 1.20 m (가정) |
| 총질량 | **74.10 kg** (베이스 33.69 · 몸통 12.06 · 팔 14.17 × 2) |

## 실행

```bash
pip install mujoco==3.15.0          # 노트북 3.14.0도 로드 확인 필요
python -m mujoco.viewer --mjcf=b1/models/v0/mjcf/scene.xml           # 영문 경로일 때
python members/park-jongjin/week04/scripts/v0_mjio.py b1/models/v0/mjcf/scene.xml   # Windows 한글 경로(바탕 화면 등)
```

- Windows에서 저장소가 한글 경로 아래 있으면 MuJoCo가 xml · STL을 직접 못 엶 (week03 발견) → `v0_mjio.py`가 xml과 STL을 메모리로 읽어 assets로 넘김 (클라우드에서 `/tmp/한글 경로/` 로 재현 · 확인)
- 다시 만들기: `step_to_links.py` → `build_v0_urdf.py` → robot.xml 재생성(`mj_saveLastXML`) → `v0_checklist_check.py` (week04 log 3절)

## 메시 · 관성 출처 (10/14 통합)

- **함태훈** [f9c9eff `sim/robot/`](https://github.com/taehoonham0118-cell/-1-/tree/f9c9eff/sim/robot) — 링크별 visual · collision STL, `<inertial>`(COM 기준 텐서), torso는 `link_mass_props.csv`의 CAD 부품값 + B1 몸통 box 가정(S11)
- 두 모델 모두 `cad_frame`과 같은 축 방향을 써서 회전 차 0 → 링크 원점만 다름(관절 축 **방향**으로 17~22 mm, 수직 1.66 mm). [merge_taehoon.py](../../../members/park-jongjin/week04/scripts/merge_taehoon.py)가 COM · 메시를 평행이동 dp만큼 옮겨 B1 골격 프레임에 맞춤 (관성 텐서는 COM 기준이라 그대로)
- right 팔은 공칭(좌우 대칭) 규칙대로 left를 거울 — 함태훈 right 실측값과 차이는 joint-origin-check · inertial 비교에 기록
- B1 임시 메시 · 관성(STEP 직접 추출)은 커밋 6a971f9 · `link_inertials_step.json`에 보관 → 대조표 [inertial_b1_vs_taehoon_v0.md](../../../members/park-jongjin/week04/results/inertial_b1_vs_taehoon_v0.md) (질량 차 ≤ 6.4 %, 밀도 가정 차)

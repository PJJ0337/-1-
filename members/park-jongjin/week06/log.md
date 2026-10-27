# week06 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 06 주차 (2026-10-26 ~ 2026-10-30), 마감 10/29(목) |
| 게이트 | **PDR (×2)** — 기획서 통과 기준 「개념설계 대안 선정 완료, 액추에이터 토크 여유 20 % 이상 확인」 |
| 커리큘럼 | 노션 「B1 5·6주차 학습 커리큘럼」 §2 — 순기구학 · 역기구학 · 궤적 추종 + 토크 예산 + Isaac 임포트 준비 (박종진 = 작업공간 · DLS IK · 임포트 대응표) |
| 저장소 태그 | `week06` (제출 시) |
| 주간 보고서 | [학부생_주간활동보고서_6주차_박종진_22212289_20261029.docx](../reports/학부생_주간활동보고서_6주차_박종진_22212289_20261029.docx) |

## 1. 계획 대비 수행

| No. | 계획 (5주차 보고서 · 커리큘럼) | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 1 | 작업공간 점구름(팔별 · 공유, ≥ 20,000) + 상판 기준 높이 · 도달거리, `workspace_max_torque.json`(함태훈 토크 예산표용) | 팔당 30,000 샘플 → 충돌 제외 29,757 · 29,775점. 상판 위 0.206 ~ 1.389 m, 전방 0.594 m(가장자리 밖 0.306 m), 공유 0.069 m³(정중면 ±0.105 m 판 — 공유 판은 추가 30만 샘플로 조밀화), 높이 구간별 도달표, 베이스 이동 확장. 관절별 정적 토크 최대 자세(0 / 0.5 / 1 / 3 kg) = 검증 ③과 일치(42.92 · 39.77) | 100 % | scripts/workspace.py, results/workspace.{md,json} · workspace_max_torque.json · workspace_points.npz, img/workspace.png |
| 2 | DLS IK `ik.py` — 인터페이스(solve_ik), 목표 10개 표, 함태훈 집기 시나리오 제공 | 위치 / 위치 + 그리퍼 축 / 자세 전체, 관절 한계 클리핑 · 스텝 제한 · 재시작, 실패 분류(unreachable · joint_limit · **orientation_limit** 추가). 목표 10개: 안 8/8 수렴, 밖 2 최근접 해. λ 5종 비교, 수렴률 500 목표 위치만 99.6 % · 위치 + 축 97.4 %, 오른팔 거울 1e-15 rad. GIF | 100 % | scripts/ik.py · ik_demo.py, results/ik_results.{md,json}, img/ik_targets.{png,gif}, docs/ik_interface.md (v1) |
| 3 | Isaac 임포트 대응표 + 패키지(메시 visual/collision, 상판 메시 여부) | 메시를 `meshes/visual` · `meshes/collision`으로 분리, URDF 메시 = `../meshes/…` 상대 경로(strippath=false), MJCF meshdir 유지 · 로더 하위 폴더 대응. 대응표(14항목) + Manager-Based 대응 + 첫날 할 일, Isaac Lab ArticulationCfg 초안, 패키지 자동 점검. 상판 메시 = 넣지 않음(box로 충분) | 100 % (실습 PC 미배정 — Isaac 쪽 ⚠ 항목은 배정 후 확인) | scripts/export_isaac.py, docs/import_settings.md, isaac/b1_v1_cfg.py, results/isaac_package_check.json |
| 4 | gravcomp.enable 켜기, A2 armature · forcerange 반영(어깨 TQ 확인 후, 함태훈과) | gravcomp on(validate ②의 보상 없음 · A는 gravcomp 뺀 모델로 계산해 week05 의미 유지). A2 사양을 `inputs/a2_actuators.json` case 두 개(a2_doc = 어깨 TQ, cad_ri85 = 8개 RI85)로 두고 **기본 a2_doc** — 어깨 TQ 확인 전이라 case 한 줄로 전환. armature 0.219 / 0.098, forcerange · URDF effort = 피크 381.7 / 181.5. case별 계단 응답 비교(yaw ω_n 140 → 35.8 rad/s). 검증 5종 재실행 통과, 체크리스트 28/28 | 90 % (팀장 어깨 구동기 확정 · 함태훈 확인 대기) | inputs/a2_actuators.json, actuator_params.json(a2 · gravcomp), build_v1.py, scripts/a2_case_compare.py, results/a2_case_compare.md |
| 5 | 금요일 평가 피드백 반영 | 5주차 평가 피드백 미수령(10/27 기준) | – | – |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| 작업공간 높이 (상판 윗면 기준) | **0.206 ~ 1.389 m** (바닥 기준 0.606 ~ 1.789) → 상판 위 0.2 m 이하 · 바닥 도달 불가, 베이스 이동으로도 높이 불변 | workspace.md |
| 최대 전방 도달 | **x 0.594 m** (z 1.22 m, 어깨 높이) = 상판 앞 가장자리 밖 0.306 m. 테이블 높이(z 0.7 ~ 0.8)에서는 0.437 m(가장자리 밖 0.149 m) | workspace.md 높이 구간표 |
| 양팔 공유 작업공간 | **0.069 m³** (왼팔 0.488의 14 %), y −0.105 ~ 0.105 m · z 0.735 ~ 1.695 m — roll 하한 −0.28 rad(몸통 폭 가정) 때문 | workspace.md, img/workspace.png |
| IK 목표 10개 | 작업공간 안 **8/8 수렴** (위치 오차 ≤ 0.88 mm, 반복 7 ~ 27), 밖 2개 success = False · 최근접 해(T9 205 mm, T10 214 mm) | ik_results.md |
| IK 수렴률 | 도달 가능 목표 500개: 위치만 **99.6 %**(반복 중앙값 14), 위치 + 그리퍼 축 **97.4 %** | ik_results.md |
| λ 영향 | 특이점 근처(T7) 원 스텝 최대 λ 0.005 → 20.3 rad · λ 0.3 → 0.74 rad, 대신 반복 14 → 133회. 기본 0.05 + 스텝 제한 0.3 rad + 재시작 4 | ik_results.md λ표, 구술 노트 05 |
| 4-DOF 자세 한계 | 위치 + 그리퍼 축(5조건 > 4자유도) — X0 ~ X2: 위치 0.000 mm, 축 1.8 ~ 21.7° 남음 → 목표 사양 6-DOF(손목 2축) 필요 근거 | ik_results.md |
| 토크 예산 입력 (정적 최악) | 0 / 0.5 / 1 / 3 kg: pitch 25.41 / 28.32 / 31.24 / **42.92**, roll 22.86 / 25.68 / 28.50 / **39.77** N·m | workspace_max_torque.json |
| PDR 토크 여유 | 3 kg × 1.2 ≤ 52 N·m 통과(pitch 21.2 %). A2 정격 대비 × 1.5 안전율: 어깨 TQ 118.8 → 여유 177 %, **어깨도 RI85(72.6)라도 69 %** → 두 case 모두 PDR 통과 | workspace.md PDR 표 |
| A2 armature 영향 | a2_doc: kv pitch 20.5 → 29.8 · yaw 4.25 → 16.8, yaw ω_n 140 → 35.8 rad/s, 정착 0.044 → 0.164 s, 수평 유지 0° | a2_case_compare.md |
| 검증 5종 (week06 모델) | ① 75.097 kg ② 수평 보상 없음 4.83° → B 0.0° ③ 42.92 · 39.77 ④ 3,770자세 0 ⑤ 0 mm, Isaac 사본 동등 5e-15, 체크리스트 28/28 | validate_model.md, checklist_v1.md |

## 3. 재현 방법

```bash
pip install mujoco==3.15.0 trimesh scipy matplotlib imageio pillow
python b1/models/v1/scripts/build_v1.py               # A2 case a2_doc · gravcomp on
python b1/models/v1/scripts/validate_model.py
python b1/models/v1/scripts/workspace.py              # ~20 s
python b1/models/v1/scripts/ik.py                     # ~6 s
python b1/models/v1/scripts/ik_demo.py                # GIF (리눅스 헤드리스: MUJOCO_GL=glfw xvfb-run -a python …)
python b1/models/v1/scripts/a2_case_compare.py
python b1/models/v1/scripts/export_isaac.py
MUJOCO_GL=glfw xvfb-run -a python b1/models/v1/scripts/render_v1.py && python b1/models/v1/scripts/checklist_v1_check.py
```

## 4. 발견한 문제 · 해결

- (10/27) 공유 작업공간을 균일 샘플 3 cm 복셀로 세면 0.008 m³ — 공유 영역이 정중면 ±0.1 m 판이라 복셀당 점 2개 안팎 → 판 안만 추가 30만 샘플(mj_kinematics + mj_collision)로 조밀화 → 0.069 m³(경계 누락이 커서 생긴 과소 추정)
- (10/27) 위치 + 그리퍼 축 IK가 「작업공간 안」 점에서 정체(가중 최소자승 국소해) → 원인 = 4-DOF라 축 고정 시 팔꿈치 위치가 정해져 해가 2차원 곡면에만 존재. 위치 우선 폴백(위치만 IK → 위치 등식 제약 SLSQP로 축 최소) + status `orientation_limit` 추가. 영공간 투영 DLS는 λ 때문에 투영이 정확하지 않아 위치가 35 mm 남아 버림
- (10/27) 「아래로 접근」 집기점을 zc > 0.999로 고르면 축이 최대 2.5° 어긋나 실현 불가 점이 됨 → elbow 1차원 최소화로 정확한 수직 접근점(0.2551, 0.2867, 0.7508) 생성
- (10/27) gravcomp를 빌드 기본 on으로 바꾸자 render_v1.py의 목표 오프셋 피드포워드가 이중 보상(−4.84°) · 액추에이터 점검 7/11 → 빌드에 gravcomp가 있으면 피드포워드 생략, 처짐 그래프는 gravcomp 뺀 모델과 비교, 토크는 actuator_force 대신 qfrc_actuator(보상분 포함) → 11/11, 0.0°
- (10/27) MuJoCo URDF 파서는 기본으로 메시 경로의 디렉터리를 버림 → `strippath="false"` + URDF는 `../meshes/visual/x.stl`, MJCF는 `meshdir="../meshes/"` + `visual/x.stl`로 다시 씀. 한글 경로 로더(assets dict) 키도 `visual/x.stl` 형태로 변경
- 5주차 평가 피드백 미수령 → 수령 시 week07에 반영

## 5. 막힌 점 · 요청 (공유한 날짜)

- (팀장) 어깨 구동기 = TQ ILM-E85x30(A2 사양서) vs RI85(CAD) 확정 요청 — 둘 다 PDR 토크 통과, 기본 case a2_doc
- (함태훈) ① actuator_params.json에 `a2` · gravcomp on 반영한 것 확인 ② kp · kv 튜닝을 a2_doc 기준으로 ③ `workspace_max_torque.json` · `ik.py` 인터페이스 v1(orientation_limit 추가) 사용 — 집기 시나리오는 위치만 IK + 연속 q_init 권장
- (A팀 피드백, PDR) 상판 위 0.2 m 이하 · 바닥 도달 불가, 테이블 높이에서 가장자리 밖 0.15 m, 공유 작업공간 폭 0.21 m, 4-DOF로 수직 하강 접근 제한 → 손목 2축 · 몸통 높이 · roll 하한(몸통 폭) 검토 요청

## 6. 구술 질문 준비 (커리큘럼 6주차)

- Q1 DLS λ ↑ / ↓ 특이점 · 정확도 / Q2 6-DOF 위치만 vs 위치 + 자세 자코비안 크기 · 해 개수 → [notes/mujoco-urdf/05-workspace-ik.md](../notes/mujoco-urdf/05-workspace-ik.md)

## 7. 다음 주 계획 (7주차, 검증 가능한 단위)

1. 팀장 어깨 구동기 확정 → case 고정 · 재빌드, 5주차 · 6주차 평가 피드백 반영
2. 함태훈 집기 시나리오 결과와 IK 연동 점검(추종 오차 · 페이로드별 토크) → 토크 예산표 B1 통합본
3. 실습 PC 배정 시 `import_settings.md` 4절 순서로 Isaac Sim 임포트 · 검증 ② ③ 재현, 미배정 시 손목 2축 추가 시 작업공간 · IK 변화 예측(A1 회신 기준)

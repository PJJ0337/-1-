# week05 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 05 주차 (2026-10-19 ~ 2026-10-23), 마감 10/22(목) — 작업 10/16(금) 선행 |
| 게이트 | 해당 없음 (다음 주 6주차 = PDR ×2) |
| 커리큘럼 | 노션 「B1 5·6주차 학습 커리큘럼」 §1 — 전체 모델 v1 + MJCF 구동 · 센서 · 검증 5종 |
| 저장소 태그 | `week05` (제출 시) |
| 주간 보고서 | [학부생_주간활동보고서_5주차_박종진_22212289_20261022.docx](../reports/학부생_주간활동보고서_5주차_박종진_22212289_20261022.docx) |

## 1. 계획 대비 수행

| No. | 계획 (커리큘럼 · 4주차 평가표 합의) | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 1 | (4주차 합의 ①) assumptions-v0 → **B1 공용 가정값 v0.1**, 함태훈 URDF와 총질량 · 수평 토크 일치 확인 | 팀장 확정값 C1~C8, 공용 json. 74.10 − 67.46 = 6.63 kg 분해(상판 +3.69 · 몸통 +3.00 · 오른팔 −0.06), 공용값 적용 시 74.093 = 74.096 kg, 22.73 / 37.30 N·m 일치. v0.2 = S14 그리퍼 추가 | 100 % | b1/models/assumptions-v0.md, assumptions_b1.json, results/b1_vs_taehoon_mass_torque.md |
| 2 | (개별) 양팔 + 베이스 **v1 URDF 생성 스크립트** + 좌우 미러 규칙표 | 왼팔 정의 → 오른팔 미러 생성(위치 y 반전, 축 = 유사벡터 x · z 반전, rpy roll · yaw 반전, 관성 ixy · iyz 반전, 메시 winding). 상판 별도 링크, 그리퍼 임시 box + ee site. 원본(planar) + Isaac 사본(prismatic · prismatic · continuous) | 100 % | b1/models/v1/scripts/build_v1.py, urdf/b1_v1.urdf · b1_v1_isaac.urdf, docs/mirror_rules.md |
| 3 | (개별) 홀로노믹 베이스 3-DOF + velocity 액추에이터, 속도 명령 vs qvel | kv 200 / 50, 시정수 0.378 s(이론 M/kv 0.375 s), yaw 회전 후 body 기준 명령 → world 변환 확인 | 100 % | scripts/base_velocity_test.py, img/base_velocity_test.png, results/base_velocity_test.json |
| 4 | (팀 연계) URDF 생성 → MJCF 변환 → 덧붙이기 **빌드 스크립트** | 덧붙이기 값을 `actuator_params.json`(함태훈 담당)으로 분리 → 재빌드해도 유지. class shoulder · elbow · base, 액추에이터 11, 센서 30(jointpos · jointvel · actuatorfrc, IMU gyro · acc, ee framepos · quat) | 100 % (함태훈 값 채우기 · 규약 합의 대기) | build_v1.py, actuator_params.json, mjcf/b1_urdf.xml · b1.xml · scene.xml |
| 5 | (개별) **검증 ④ · ⑤** (+ ① · ② · ③ 공용 기준 계산 → 함태훈 결과 대조용) | ① 75.097 kg (+0.004 %) ② 영점 0.045°, 수평 4.83° → 중력보상 0.000° ③ 3 kg pitch 42.91 · roll 38.89 N·m → 정격 46 기준 미통과(그리퍼 가정 영향) ④ 3,770자세 접촉 0 ⑤ 504자세 차 0 mm. Isaac 사본 동등(1e-15) | 100 % | scripts/validate_model.py, results/validate_model.md · .json |
| 6 | (개별) 체크리스트를 v1 기준으로 갱신 | v0.4: 0-3절 빌드 규약(제안), E7-1 중력보상, E8 액추에이터 응답, F6 검증 5종, M절 미러. 자동 점검 28항목 OK | 100 % | b1/docs/procedures/mujoco-urdf-loading-checklist.md, v1/results/checklist_v1.md |
| 7 | (4주차 합의 ③) 중력보상으로 수평 처짐 상쇄 · 캡처 | 목표 + τg/kp 피드포워드: 4.83°(이론 4.85°) → 0.000°, 토크 25.40 N·m 동일 | 100 % | scripts/render_v1.py, img/v1_gravcomp.png, results/gravcomp.json |
| 8 | (팀장 10/16 ④) RI85 요청서 §2 J1~J4 × 양팔 8행 | STEP 구동기 번호 · 시뮬 범위 · 축/구동기 +방향 · 비고 채움, 감속비 · 토크 · 스토퍼는 요청 | 100 % | b1/docs/icd/actuator-spec-request.md (v0.2) |
| 9 | (4주차 합의 ②) A1 · A2 회신 반영 v1 재생성 | 회신 대기 — 가정값 유지. 수령 시 json만 바꾸고 `build_v1.py` 재실행 | 0 % | – |
| 9-1 | (커리큘럼 수) 뷰어에서 액추에이터 슬라이더 정상 동작 캡처 | (10/16) 노트북 Windows · 한글 경로, MuJoCo 3.14 → 3.15.0 업그레이드 후 v1_mjio.py: 75.097 kg 로드, 영점 정지, 양팔 shoulder_pitch 3 rad 대칭, base_vx 0.195 m/s 전진 | 100 % | b1/models/v1/img/repro/20261016_laptop_v1_*.png |
| 10 | 구술 Q1 · Q2 대비 | 미러 축 판별(유사벡터), 평면 근사의 손실 · 정당화 + 수치 | 100 % | notes/mujoco-urdf/04-v1-mirror-base.md |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| v1 모델 | nq = nv = 11, nu 11, nbody 18, 센서 30, 총 **75.10 kg** (v0 74.10 + 그리퍼 0.5 × 2) | results/checklist_v1.md F1 · F2 |
| 검증 ① 질량 | 75.097 vs 기준 75.094 kg (+0.004 %), 링크별 0 % (기준 = 공용 가정값 + 함태훈 CAD, BOM 대기) | validate_model.md ① |
| 검증 ② 정지 유지 5 s | 영점 0.045°, 팔 수평 보상 없음 4.83° (이론 −bias/kp 4.85°) → **중력보상 0.000°**, 떨림 0 | validate_model.md ②, img/v1_gravcomp.png |
| 검증 ③ 최악 중력 토크 (15° 격자 전수) | 0 kg: pitch 25.40 · roll 22.25 · yaw 3.04 · elbow 2.99 / **3 kg: 42.91 · 38.89 · 11.84 · 11.78 N·m** | validate_model.md ③ |
| ③ 판정 | 정격 46 N·m 가정 시 pitch 여유 **7.2 %** (< 20 %) → 필요 정격 51.5 N·m. 원인 = 그리퍼 0.5 kg(+2.7) + 페이로드 점 플랜지 → ee 0.10 m(+2.9) | assumptions-v0.md §4 |
| 검증 ④ sweep | 8관절 0.5° 간격 3,770자세 범위 안 접촉 0, 한계 밖 첫 접촉: roll −2° · elbow −3° / +4° | validate_model.md ④ |
| 검증 ⑤ 대칭 | 504자세 \|Δx\| · \|Δz\| · \|y_L + y_R\| = 0 mm, 영점 ee (0, ±0.2866, 0.605) m | validate_model.md ⑤ |
| 손계산 = MuJoCo (F3) | 25.4035 = 25.4035 N·m (그리퍼 포함) | checklist_v1.md F3 |
| 베이스 응답 | vx · vy 63 % 0.378 s (M/kv = 75.1/200 = 0.375 s), yaw 0.082 s, 정상상태 오차 0.005 m/s(2 s 구간) | base_velocity_test.json |
| 액추에이터 응답 | 11/11 — 범위 40 % 명령에 position 오차 0, velocity 98 % (1.5 s) | actuator_check.json |
| Isaac 사본 | prismatic · prismatic · continuous + 가상 링크 2 g, 원본과 50자세 차 3.6e-15 | validate_model.md 보조 |

## 3. 재현 방법

```bash
pip install mujoco==3.15.0 trimesh scipy matplotlib
python b1/models/v1/scripts/build_v1.py
python b1/models/v1/scripts/validate_model.py
python b1/models/v1/scripts/base_velocity_test.py
MUJOCO_GL=osmesa python b1/models/v1/scripts/render_v1.py      # Windows는 MUJOCO_GL 없이
python b1/models/v1/scripts/checklist_v1_check.py
python b1/models/v1/scripts/v1_mjio.py b1/models/v1/mjcf/scene.xml   # 뷰어 (한글 경로 대응)
python members/park-jongjin/week05/scripts/compare_b1_taehoon.py b1/models/v0/urdf/b1_humanoid_v0.urdf <함태훈 our_robot_v0.urdf> members/park-jongjin/week05/results/b1_vs_taehoon_mass_torque.md
```

## 4. 발견한 문제 · 해결

- (10/16) 커리큘럼은 URDF 베이스를 prismatic 2 + revolute 1로 쓰라고 하고, 팀장 10/16 결정은 planar 1개 유지 → **원본 = planar(팀장), Isaac용 사본 = 3관절(커리큘럼 · C3-1)** 둘 다 빌드에서 생성, 동등성 1e-15로 확인
- (10/16) site는 body 안에 있어야 해서 `include` 덧붙이기(0-2절 3단 규약)로 넣을 수 없음 → 빌드가 변환본을 읽어 b1.xml을 생성(변환본은 그대로). 체크리스트 0-3절에 규약 변경 제안
- (10/16) 메시를 y 반전만 하면 면 법선이 안쪽 → winding 뒤집기 추가 (체크리스트 M2)
- (10/16) 검증 ③ 첫 격자가 범위 하한에서 시작해 −90° 자세를 놓침(42.79 vs 42.91 N·m) → 0° 기준 15° 격자로 수정
- (10/16) 그리퍼 · 페이로드 작용점을 넣자 정격 46 N·m 기준이 깨짐(필요 51.5 N·m) → 팀장 보고, 그리퍼 사양 요청
- (10/16) 중력보상 처짐 그래프에서 이론선 부호 반대(qfrc_bias = 중력을 상쇄하는 힘) → 이론 처짐 = −bias/kp로 수정
- tools/new_week.sh CRLF로 VM bash에서 `pipefail` 오류 → log 직접 작성

## 5. 막힌 점 · 요청 (공유한 날짜)

- (팀장, 10/16) A1 편차 의도 · +z 정면, A2 RI85 사양 — 회신 대기
- (팀장, 10/16 예정) 그리퍼 질량 · 치수 · TCP 위치(BOM) — S14 0.5 kg 가정, 검증 ③ 판정에 직접 영향
- (함태훈, 10/16) 공용값 v0.1 적용 · v1 메시(관절점 원점) · `actuator_params.json` 값 · 0-3절 빌드 규약 합의, 검증 ① · ② · ③ 함태훈 결과와 대조

## 6. 구술 질문 준비 (커리큘럼 5주차)

- Q1 좌측 팔 미러 시 axis 부호 바꾸는 관절 구분 / Q2 옴니휠 → slide · slide · hinge 근사로 잃는 것과 괜찮은 이유 → [notes/mujoco-urdf/04-v1-mirror-base.md](../notes/mujoco-urdf/04-v1-mirror-base.md)

## 7. 다음 주 계획 (6주차, 검증 가능한 단위)

1. 작업공간 점구름(팔별 · 공유, 샘플 ≥ 20,000) 산점도 + 상판 기준 높이 · 전방 도달거리
2. DLS IK `ik.py` (위치 · 위치 + 자세, 관절 한계) — 목표 10개 수렴 표, 함태훈 집기 시나리오에 함수 제공
3. Isaac 임포트 대응표 `import_settings.md` — 마감 10/29(목), week06

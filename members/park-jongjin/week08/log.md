# week08 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 08 주차 (2026-11-09 ~ 2026-11-13), 마감 11/12(목) |
| 커리큘럼 | 노션 「B1 7·8·9주차 학습 커리큘럼」 §2 — 접촉이 있는 집기 시나리오 + 시연 데이터 수집 파이프라인 (박종진 = 상태기계 · 성공 판정 · 무작위화 · 수집 스크립트 · 실패 분류) |
| 저장소 태그 | `week08` (제출 시) |
| 주간 보고서 | [학부생_주간활동보고서_8주차_박종진_22212289_20261112.docx](../reports/학부생_주간활동보고서_8주차_박종진_22212289_20261112.docx) |
| 함태훈 대응 | taehoonham0118-cell/-1- — 11/12 기준 week07 `59a5d80`까지(week08 그리퍼 · 접촉 scene · `docs/dataset_spec.md` 미게시) → 그리퍼 · 데이터 형식은 **B1 임시본**으로 진행, 받으면 교체 |

## 1. 계획 대비 수행

| No. | 계획 (7주차 보고서 · 커리큘럼) | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 1 | 시나리오 상태기계 + 성공 판정 + 무작위 20회 성공률 · 실패 분류 (전문가 = wb_ik + resolved-rate, 옆면 · 수직 하강) | 10단계 상태기계(측정값 전환 · timeout), 전신 자세 IK(위치 3 + 자세 3, 7-DOF) + 전신 resolved-rate 직선 구간, 실패 10분류. **무작위 20회 19 성공(95 %)**, 실패 1 = ik_fail(상자 · 깊이 0.212 m · yaw 40°). 깊이별 계획 성공(원기둥 0.24 m까지 7/7, 0.26 m 0/7) | 100 % | scenarios/{pick_env,expert,run_pick20}.py, results/pick20.{md,json} |
| 2 | wb_ik 팔 링크 ↔ 테이블 충돌 확인 | `Ref.table_hits`(mj_geomDistance, 로봇 geom 16개 ↔ 테이블): 정지 자세 20 mm(손가락 파지 자세 5 mm · 베이스 20 mm), 관절공간 이동 경로 30점 50 mm(서보 지연으로 실제 손이 기준보다 낮게 지나감) → 경유점 거리 0.30 · 0.45 · 0.60 · 0.75 m 중 첫 무충돌. **실행 중 테이블 접촉 0회**(20회 · 100회) | 100 % (함태훈과 결과 공유 예정) | expert.py `table_hits` · `path_hits` |
| 3 | 데이터 수집(함태훈 dataset_spec) 100 에피소드 · 재생 검증 · 샘플 3개 | 함태훈 명세 미게시 → B1 초안 v0.1(HDF5, obs · action 8 · ctrl · meta). **100개 99 성공**, 12.2만 스텝, 44.6 MB(커밋 안 함). **재생 100/100 qpos 최대 차 0**. 샘플 3개(카메라 96 × 96) 커밋 | 100 % (형식은 함태훈 명세로 맞출 예정) | scenarios/collect.py, datasets/dataset_spec_draft.md · samples/, results/collect.{md,json} |
| 4 | (커리큘럼 월 · 함태훈 몫 임시) 그리퍼 · 접촉 scene, 마찰 × 파지력 × 질량 미끄러짐 표 | 평행 2지(slide 2 + equality, 손가락당 힘 상한 40 N, 가정값) + 테이블 · 상자 · 원기둥. 54조합 이론(F ≥ m g / 2μ)과 **54/54 일치**(elliptic + impratio 10), MuJoCo 기본 pyramidal 38/54(크리프), condim 3 · 4 · 6 비교 | 100 % (임시) | scenarios/gripper_slip.py, results/gripper_slip.md |
| 5 | 집기 GIF 1개 | 상자 2.90 kg seed 6 — 접근 · 하강 · 파지 · 이동 · 놓기 · 복귀(24.8 s) | 100 % | scenarios/img/pick_seed6.gif |
| 6 | 어깨 확정 · 평가표 반영 | 팀장 회신 · 5 ~ 7주차 평가표 미수령 → a2_doc 유지 | 0 % (외부 대기) | — |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 | 값 | 근거 |
|---|---|---|
| 집기 성공률 (무작위 20회) | **19/20 = 95 %**, 목표 오차 평균 6.0 · 최대 13.5 mm, 2 kg 이상 3/3, 정렬 오차 평균 2.0 mm, 에피소드 24.5 s | results/pick20.md |
| 실패 분류 | ik_fail 1(상자 면 방향 + 깊이 경계), 나머지 9분류 0 | results/pick20.md |
| 도달 한계 (수직 하강 · 옆면 · 베이스 여유 40 mm) | 원기둥 테이블 안쪽 0.24 m까지 7/7, 0.26 m 0/7 (7주차 끝점 기준 0.306 m) | results/pick20.md |
| 그리퍼 파지 유지 | 이론과 54/54 일치, F ≥ 1.5 F_min 최대 미끄러짐 0.97 mm / 기본 원뿔 38/54 · 24 mm | results/gripper_slip.md |
| 수집 | 100 에피소드 · 99 성공 · 122,400 스텝(50 Hz) · 44.6 MB · 103 s | results/collect.md |
| 재생 일치 | 100/100 qpos 최대 차 **0.0**(비트 일치), 성공 판정 일치 100/100, 모델 지문 일치 100/100 | results/collect.md |
| 테이블 충돌 | 계획 단계 점검 후 실행 중 로봇 ↔ 테이블 접촉 0회 | results/pick20.md |

가정값: 그리퍼(스트로크 55 mm/손가락, 손가락 0.2 kg, 힘 상한 40 N, 파지 중심 = 손바닥 아래 45 mm), 마찰 1.0(손가락 · 물체) · 0.8(테이블), 테이블 0.70 m, 물체 0.2 ~ 3 kg, 어깨 구동기 a2_doc case

## 3. 재현 방법

```bash
pip install mujoco==3.15.0 numpy scipy h5py pillow
python scenarios/gripper_slip.py                    # ~5 s
python scenarios/run_pick20.py                      # ~65 s
python scenarios/collect.py --n 100 --start 1000    # ~100 s → datasets/raw/ (커밋 안 함)
python scenarios/collect.py --replay datasets/raw   # ~55 s
MUJOCO_GL=egl python scenarios/render_pick_gif.py 6 # Windows는 그냥 python
```

## 4. 발견한 문제 · 해결 (성공률 2/20 → 19/20 경과)

- (11/12) 처음 2/20: APPROACH 관절 보간 중 **손가락이 테이블을 쓸고 지나감**(팔 내린 자세 → 테이블 위 자세) → 경유점(베이스를 0.30 m 뺀 자리에서 팔 자세 변경) + 경로 30점 테이블 거리 점검
- (11/12) 베이스가 목표를 4 ~ 6 cm 넘어가 테이블에 부딪힘 → 원인 ① 속도 액추에이터 1차 지연(τ 0.375 s, 함태훈 week05 측정) → 앞먹임 v = v_ref + τ a_ref + K e ② resolved-rate에서 베이스를 테이블 제한으로 잘라도 **속도는 안 잘라** 앞먹임이 계속 밀어냄 → 잘린 뒤 실제 변화량으로 속도 재계산. 테이블 여유 20 → 40 mm
- (11/12) 수직 하강 도중 베이스가 테이블 쪽으로 가야 하는데(4-DOF) 접근 자세가 이미 한계 → 하강 못 끝냄 → **파지 자세를 먼저 풀고 위로 올려 접근 자세**
- (11/12) 전신 IK가 정면 근처 국소해(yaw ≈ −100°)에 갇혀 깊이 0.16 m에서 막힘 — 7주차 해석은 0.30 m → **수직 하강 자세 라이브러리**(7주차 workspace_wb ④와 같은 투영 18,471자세)에서 옆면 기준 시작값 8개 → 0.24 m
- (11/12) MOVE에서 팔이 거울 가지로 뒤집혀(pitch −1.48 → +1.47) 물체 낙하 → 놓기 계획 시작값 = 지금 자세 평행이동, 라이브러리도 지금 팔과 가까운 가지 우선
- (11/12) 그리퍼 유지 시험 전부 낙하 → mocap 손바닥은 속도 0이라 마찰이 안 걸림 → slide 서보. 그다음 F ≫ F_min에도 크리프 → elliptic + impratio 10
- (11/12) **pick_env의 option 덧붙이기 정규식이 scene.xml과 안 맞아 elliptic이 실제로는 안 들어가 있었음**(impratio 1 출력으로 발견) → `<option …/>` 일반 치환 + assert
- (11/12) 2 ~ 3 kg 물체 들기 · 놓기에서 MuJoCo 발산(BADQACC) → 손가락 0.05 → 0.2 kg, 그리퍼 명령 변화율 0.06 m/s, dt 2 → 1 ms. 남은 1/100은 `sim_unstable` 분류(table_collision으로 잘못 세던 것 수정)
- (11/12) 데이터 모델 지문이 재생 때 달라짐 → reset이 물체 질량을 모델에 써서 → 지문은 build 직후 1회

## 5. 막힌 점 · 요청 (공유한 날짜)

- (함태훈에게 11/12) ① week08 그리퍼 · 접촉 scene과 `docs/dataset_spec.md`가 아직 없어 B1 임시본(pick_env.GRIPPER · CONTACT, dataset_spec_draft.md)으로 수집 — 형식 · 필드 이름 맞춰 줄 것, 받으면 `save()`만 바꿔 재수집(2분) ② 실패 사례 분석 요청: seed 1080(원기둥 2.98 kg 들기 순간 발산), 파지력 · 손가락 질량 · dt 민감도 → solref · solimp 원인 표 ③ 파지 중 토크 예산: 3 kg + 그리퍼 0.9 kg, 옆면 접근 자세 roll ④ action = 관절 목표(재생 정확) vs 끝점 목표 — 함태훈 구술 ② 주제와 연결
- (팀장) 어깨 구동기 TQ ↔ RI85 확정 · (평가) 5 ~ 7주차 평가표 — 대기

- **(함태훈 11/12 회신 · 태그 week08 = 7f434ae, 반영은 제출 후 → week09 태그)**
  ① 그리퍼 · 접촉 = 임시값 그대로 채택, **OPTION timestep 1 → 0.5 ms · NSUB 20 → 40**(제어 50 Hz 유지) → seed 1000 ~ 1099 100/100(1080 포함), 계산 1.9배
  ② 명세 `docs/dataset_spec.md` v1.0(`b1-dataset-1.0`): 초안 필드 유지 + 로봇 좌표 파생 4개(obs/base_pose · ee_pos_base · obj_pos_base, action_base_body) · 속성 5개 · 성공 시 fail = "" — `b1_dataset.upgrade()`/`check()` → `datasets/`에 사본, collect.py가 초안으로 쓴 뒤 upgrade · check
  ③ seed 1080 원인: 미끄러짐이 아니라 **수치 발산**(들기 10.86 s부터 물체 각속도 0.1 → 17 rad/s 지수 증가). solref · solimp 딱딱 → 발산 증가(0.005: 11건), 부드럽게 → 관통 · 낙하 증가, impratio ↓ → 발산 증가, condim 무관. **dt 0.5 ms만 해결**(3 kg 고정 20회 9 → 19/20, 원기둥 2/12 → 12/12). 손가락 0.2 kg · 닫기 속도 · dt 2 → 1 ms 조치는 같은 원인을 줄이는 쪽이라 유지
  ④ 파지 토크: 3 kg @ 파지 중심 + 손가락 0.4 kg 정적 최악 pitch 46.53 · roll 42.52, + 베이스 가속 → **필요 48.69 · 46.71 N·m**, 옆면 집기 실행 최대 roll 8.7 N·m. TQ 144 % · RI85 49 % 통과, **PDR 52 pitch 여유 6.8 %(× 1.2 미달)** → `torque_budget_b1.md` 갱신(+ A팀 추가 문안 초안)
  ⑤ action = **관절 목표 확정**(재생 차 0). 끝점 목표 + IK는 직전 해 시작이면 10/10 · 관절 차 ≤ 2.84°지만 이력 없이 풀면 상자 시연 62 ~ 64 %가 다른 해 → 9주차 BC에서 두 출력 비교

## 6. 구술 질문 준비 (커리큘럼 8주차)

- Q1 상태기계 전환 조건 · 실패 분류 / Q2 4-DOF 그리퍼 축 문제와 우회(접근 방향 · 베이스 yaw) → [notes/mujoco-urdf/07-contact-grasp.md](../notes/mujoco-urdf/07-contact-grasp.md)

## 7. 다음 주 계획 (9주차, 검증 가능한 단위)

1. BC 파이프라인: 데이터 로더(HDF5 → obs · action 정규화, 학습 / 검증 분리) · MLP(CPU) · 롤아웃 성공률 — 전문가 95 % 대비, 랜덤화 없음 / 있음
2. Isaac Lab 환경 명세 `docs/isaaclab_env_spec.md` — 상태기계 · 판정 · 실패 분류 → Observation · Action · Termination · Event 대응, 함태훈 Event 매핑 통합
3. 손목 2축 예측 1쪽 — 이번 주 경계(깊이 0.24 m · 상자 방향) 기준으로 수렴률 · 성공률이 어떻게 달라질지
4. 함태훈 명세 · 접촉 분석 받으면 형식 맞춰 재수집 · sim_unstable 원인 반영

## 8. 제출 후 반영 (11/12 저녁, 함태훈 7f434ae 회신 — week09 태그에 포함, week08 태그 da55741은 그대로)

| 항목 | 변경 | 결과 | 근거 |
|---|---|---|---|
| 시뮬 dt | `pick_env.OPTION` timestep 0.001 → 0.0005, `expert` NSUB 20 → 40 | 무작위 20회 · 그리퍼 점검 · 수집 재실행(아래) | scenarios/results/*.md |
| 데이터 형식 | 함태훈 v1.0 명세 · 도구 사본(`datasets/dataset_spec.md` · `b1_dataset.py`), collect.save = 초안 → upgrade → check | 100개 재수집 · v1.0 검사 · 재생 | results/collect.md |
| 토크 예산 | `inputs/torque_budget_w08_taehoon.csv` 추가, 8주차 필요 = max(7주차, 정적 + 베이스 가속, 집기 실행) | pitch 48.69 · roll 46.71 N·m, PDR 52 pitch 6.8 % · roll 11.3 % NG | b1/models/v1/results/torque_budget_b1.md |
| action | 관절 목표 유지(합의) | — | 함태훈 sim/w08/results/action_format.md |

- 재실행 결과(dt 0.5 ms): 무작위 20회 **19/20** 그대로(실패 seed 1 상자 ik_fail — 계획 단계라 dt 무관), 목표 오차 평균 6.0 · 최대 13.6 mm / 그리퍼 54조합 이론 일치 54/54 그대로 / **수집 100/100 성공**(seed 1080 포함), 123,040 스텝 · 160 s(1 ms 103 s의 1.6배) / **v1.0 check 100/100 통과 · 재생 qpos 차 0 100/100**, 모델 지문 7016330767f5 → 4d070b18a640 / 샘플 3개 v1.0으로 다시 생성

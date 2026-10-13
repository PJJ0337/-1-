# week04 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 04 주차 (2026-10-12 ~ 2026-10-16), 마감 10/15(목) |
| 게이트 | 해당 없음 |
| 제출일 | 2026-10-15 (작업 완료 10/13) |
| 저장소 태그 | `week04` |
| 주간 보고서 | [학부생_주간활동보고서_4주차_박종진_22212289_20261015.docx](../reports/학부생_주간활동보고서_4주차_박종진_22212289_20261015.docx) |

## 1. 계획 대비 수행

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | STEP 링크 분할표 — ICD 이름 {side}_{부위}_{운동} ↔ STEP 부품 ↔ BOM (액추에이터 내부는 한 강체) | STEP 10/13 수령(카톡). OCCT로 트리 · 접촉 판정 → 팔당 링크 4 + torso, RI85는 고정측/출력측 분리. 관절 축 4개 추출. **팔 4-DOF(RI85 ×4)** 확인 | 100 % (BOM 열 대기) | b1/models/link-split-v0.md, week04/results/link_split_parts.csv, img/20261013_link_split.png |
| 2 | (개별) 골격 URDF(link · joint · limit, 베이스 planar) + 구동기 effort · velocity · armature 표(가정값 표 포함), 체크리스트 점검 | 구동기 ICD 요청서 v0 (10/13 오전). 골격 URDF: odom →planar→ base → torso → 팔 4관절 × 2 → tool0, link · joint · limit + CAD 관성(형상 없음). 구동기 표 + **가정값 표 S1~S13**(값 · 출처 · 교체 항목), 범위는 충돌 스윕으로 근거. 체크리스트 v0.3 자동 점검 **25항목 OK** | 100 % (값은 가정) | b1/models/v0/urdf/b1_humanoid_skeleton.urdf, b1/models/assumptions-v0.md, results/checklist_v0.md, b1/docs/icd/actuator-spec-request.md |
| 3 | 함태훈 메시 · inertial 합쳐 v0 URDF(3단 규약) MuJoCo 로드 · 캡처, 구술 대비 | 함태훈 STL · 관성 표 미수령(10/13 기준) → **B1 임시본을 직접 생성**: STEP에서 링크별 visual(2만 면 단순화) · collision(convex hull) STL, 재질 가정 관성. v0 URDF → robot.xml(mj_saveLastXML) → robot_actuated.xml → scene.xml 로드 · 4자세 캡처, 접촉 0. 링크 1개 inertial 비교(OCCT vs trimesh). 구술 노트 작성 | 90 % (함태훈 검증 반영, 관성 한 벌 통합은 week05) | b1/models/v0/, img/20261013_v0_mujoco.png, results/inertial_compare_left_elbow.md, notes/mujoco-urdf/03-cad-to-urdf.md |
| + | (커리큘럼 준비물) FreeCAD 1.0 · trimesh 설치, STEP 열기 | FreeCAD 1.0.2 설치(공식 해시 일치) → `양팔로봇 최종버전.stp` 열기 성공 (10/13) · 노트북 pip 확인: trimesh 5.1.0 · numpy 2.5.3 · cadquery-ocp 8.0.1.1.0(클라우드와 동일 버전) | 100 % | img/20261013_freecad_step_open.png, img/20261013_pip_install.png |
| + | (10/8 회신) 체크리스트 v0.3 — balanceinertia 기본 off | 완료 (10/13) | 100 % | b1/docs/procedures/mujoco-urdf-loading-checklist.md |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 (문서 링크 · 실행 로그) |
|---|---|---|
| STEP 최상위 인스턴스 / RI85 수 | 38 / 8 (팔당 4 → 4-DOF, 목표 6-DOF와 차이) | results/step_tree_arm.txt |
| 체적 합 | 10.22 L (팀장 미리보기 10.1 L) | results/link_split_summary.txt |
| 관절 축 (left, CAD mm) | J1 (51.6,0,0)+x · J2 (159.5,−20,41.2)+z · J3 (159.5,−202.3,−2.2)−y · J4 (159.6,−296.1,38.3)+z | results/joint_axes.txt |
| 상완 · 전완 길이, 리치 | 276.1 · 198.9 mm, J1축→플랜지 495 mm | link-split-v0.md §3 |
| 조립 이상 | J3·J4 축 1.4° 기울어짐, right 상완 이하 x 7 mm 비대칭, encoder cover 3개 누락 | link-split-v0.md §4 |
| v0 모델 | nq = nv = 11 (planar 3 + 팔 8), nu 11, nbody 14, 총 75.80 kg (팔 14.81 kg × 2, RI85 1개 ≈ 2.67 kg 추정) | checklist_v0.md F1 · F2 |
| 필요 토크 (팔 앞 수평) | 23.8 N·m (0 kg) / **38.4 N·m (3 kg)** — 손계산 = MuJoCo. 100 N·m 가정 대비 +161 % → RI85 피크 ≥ 46 N·m면 PDR 20 % 충족 | assumptions-v0.md §4, results/v0_analysis.json |
| position 처짐 | 0.0791 rad (이론 τg/kp = 0.0794, kp 300) | checklist_v0.md E7 |
| 충돌 스윕 | roll 내전 19°(몸통 box 가정), elbow ±108° (hull 기준, 보수적) → 범위 −0.28 ~ 1.5 / ±1.83 rad | assumptions-v0.md §3 |
| inertial 비교 (left_elbow_link) | OCCT 2.2677 kg vs trimesh 2.2630 kg (0.21 %), 관성 ≤ 0.78 %, COM 0.20 mm (v0.1, 함태훈 2.268 kg) | results/inertial_compare_left_elbow.md |

## 3. 재현 방법

```bash
pip install cadquery-ocp numpy matplotlib   # OCP 8.0.1 (Linux 클라우드에서 실행)
cd members/park-jongjin/week04/scripts
python step_tree.py "<양팔로봇 최종버전.stp>" arm.pkl > ../results/step_tree_arm.txt
python step_joint_axes.py "<stp>" > ../results/joint_axes.txt
python step_contacts.py "<stp>" R ; python step_contacts.py "<stp>" L   # 각 ~5분
python step_link_split.py "<stp>" > ../results/link_split_summary.txt  # link_split_parts.csv 생성
python step_link_split_plot.py "<stp>" ../img/20261013_link_split.png
# v0 모델 (값: assumptions_v0.json)
python step_to_links.py "<stp>" out_links                  # 링크별 STL · 관성 (~45 s)
python build_v0_urdf.py out_links ../../../../b1/models/v0  # skeleton · v0 URDF + meshes
python -c "import mujoco as m; M=m.MjModel.from_xml_path('../../../../b1/models/v0/urdf/b1_humanoid_v0.urdf'); m.mj_saveLastXML('../../../../b1/models/v0/mjcf/robot.xml',M)"
python v0_analysis.py ../../../../b1/models/v0/mjcf/scene.xml ../results
python v0_checklist_check.py ../../../../b1/models/v0 ../results/checklist_v0.md
MUJOCO_GL=osmesa python v0_render.py ../../../../b1/models/v0/mjcf/scene.xml ../img
python inertial_compare.py "<stp>" left_elbow_link ../results/inertial_compare_left_elbow.md
python v0_mjio.py ../../../../b1/models/v0/mjcf/scene.xml     # Windows 한글 경로 뷰어 (사용자 폴더에서 로드 확인 nq 11)
```

## 4. 발견한 문제 · 해결

- (10/13) 노션 커리큘럼 사본 페이지의 「배포 파일」에 STEP 첨부가 없음 → 원본 페이지에서 수령 필요
- (10/13) STEP을 팀장에게 카톡으로 요청 → 수령. **팔 RI85 4개 = 4-DOF**(기획서 6-DOF), 몸통 없음, 전방(±z) 판단 불가 → 링크 분할표 6절로 팀장 확인 요청
- (10/13) FreeCAD 1.0.2 STEP 가져오기: 한글 부품명이 `\X2\C591D314…` 그대로 보임(STEP 유니코드 이스케이프 미변환, CodePage 옵션 무관). 형상·트리는 정상 → 부품명은 OCCT 스크립트 결과(step_tree_arm.txt)로 대조. 가져오기 시 'Ignore instance names' 해제해야 인스턴스 번호 보존
- (10/13) OCP 8.0: `TDF_LabelSequence` 대신 `OCP.collections.Sequence_TDF_Label`, `Bnd_Box.Get()` 대신 CornerMin/Max — 버전 차이
- (10/13) MuJoCo URDF 로드: visual은 group 1 · contype 0, collision은 group 0으로 자동 분리됨 (D4 자동 충족). 오프스크린 렌더 기본 버퍼 640×480 → scene `offwidth/offheight` 지정
- (10/13) trimesh 관성: 부품 2개가 watertight 아님 → `is_watertight` 분기로 0이 들어가 관성 22~34 % 차이 → 발산정리 적분(`mass_properties`)은 닫힘 여부와 무관하게 계산되므로 그대로 쓰니 0.2 % 이내로 일치
- (10/13) 한글 경로 메시 로드: xml뿐 아니라 STL도 assets로 넘겨야 함(`v0_mjio.py`), 키는 파일명만 (meshdir 접두어와 동시 등록 시 'Repeated file name' 오류)
- (10/13 함태훈 검증 반영 — 태그 후) ① 골격 관절: 어깨 pitch · roll 앵커 ≤ 0.05 mm · 축 0.01° 일치, yaw · elbow 1.37° · 7.4 mm는 이상화로 의도된 차이 ② **RI85 출력측 = output-v1 + ring2 + cover plate1**로 통일(OCCT 재확인) → elbow 2.1425 → 2.2677 kg(태훈 2.268) ③ left yaw 축 −z → +z (태훈과 일치), right는 거울 유지 — 규약 최종은 팀장 확인 (assumptions-v0 §6) ④ ICD 요청서 v0.1: 출력축 기준 반사관성(4-1) · 효율 · 백래시(11) · 밀도 보정 근거(7) 추가
- (10/13) `.gitignore`에 `*.step` · `*.stp` · `*.SLDPRT` · `*.zip` · `bom*` 추가 (원본 커밋 금지 규칙)

## 5. 막힌 점 · 요청 (공유한 날짜)

- (10/13 팀장) 4-DOF CAD 공유, +z 전방 여부, 팔꿈치 영점 정의, CAD 조립 이상 A1 전달 여부 — 링크 분할표 6절 · 카톡
- (10/13 팀장) C3 평면 베이스 채택 — 「확인 후 연락」 대기 (v0는 planar로 작성)
- (함태훈) 링크별 STL · 질량/관성 비교표 → 수령 시 v0 관성 교체 · 대조 (B1 임시본은 cross-check용으로 공유)
- RI85 사양(피크 토크 · 속도 · 감속비 · 로터 관성) — actuator-spec-request.md, 함태훈 검토 후 발송

## 6. 구술 질문 준비 (커리큘럼 4주차)

- Q1. joint origin을 CAD 좌표에서 뽑을 때 부모 링크 프레임 기준으로 바꾸는 절차는
  - 답변 요지: 관절 점 · 축을 CAD(출력 디스크 대칭축)에서 → ICD 좌표로 회전(R) · m 변환 → 링크 원점 = 그 관절 점, 영점 rpy 0 → origin = 자식 관절 점 − 부모 관절 점 (일반식 R_pᵀ(p_c − p_p)). 검증: tool0 (0.495, 0.278, 1.191) = 리치 495 mm ✓ → [notes/03](../notes/mujoco-urdf/03-cad-to-urdf.md)
- Q2. BOM의 정격 토크와 피크 토크 중 effort에 무엇을 쓰고, MuJoCo forcerange · ctrlrange와 어떻게 대응되는가
  - 답변 요지: effort = **피크**(시뮬 힘 제한 = 물리 최대) → actuatorfrcrange + forcerange, 정격 = 제어기 · 안전 리밋. ctrlrange = 명령 각도(관절 range 상속) ≠ forcerange(힘). 필요 38.4 N·m → 피크 ≥ 46 N·m → [notes/03](../notes/mujoco-urdf/03-cad-to-urdf.md), [actuator-spec-request.md §3](../../../b1/docs/icd/actuator-spec-request.md)

## 7. 다음 주 계획 (검증 가능한 단위)

1. 함태훈 링크별 질량 · 관성표와 B1 임시 관성(link_inertials_step.json) 링크별 대조표 → 차이 5 % 넘는 링크 원인 기록, v0.1 교체
2. 팀장 회신(4-DOF · 전방 · 영점 · 베이스) 반영해 v0.1 URDF 재생성 + 체크리스트 재점검 (25항목)
3. 노트북(MuJoCo 3.14.0, Windows 한글 경로)에서 v0_mjio.py로 뷰어 실행 · 관절 슬라이더 캡처

# week04 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 / 기간 | 제 04 주차 (2026-10-12 ~ 2026-10-16), 마감 10/15(목) |
| 게이트 | 해당 없음 |
| 제출일 | (10/15 예정) |
| 저장소 태그 | `week04` |
| 주간 보고서 | [reports/…docx](../reports/) |

## 1. 계획 대비 수행

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | STEP 링크 분할표 — ICD 이름 {side}_{부위}_{운동} ↔ STEP 부품 ↔ BOM (액추에이터 내부는 한 강체) | STEP 10/13 수령(카톡). OCCT로 트리 · 접촉 판정 → 팔당 링크 4 + torso, RI85는 고정측/출력측 분리. 관절 축 4개 추출. **팔 4-DOF(RI85 ×4)** 확인 | 100 % (BOM 열 대기) | b1/models/link-split-v0.md, week04/results/link_split_parts.csv, img/20261013_link_split.png |
| 2 | (개별) 골격 URDF(link · joint · limit, 베이스 planar) + 구동기 effort · velocity · armature 표(가정값 표 포함), 체크리스트 점검 | 구동기 ICD 요청서 v0 작성 (10/13) | % | b1/docs/icd/actuator-spec-request.md |
| 3 | 함태훈 메시 · inertial 합쳐 v0 URDF(3단 규약) MuJoCo 로드 · 캡처, 구술 대비 | | 0 % | |
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

## 3. 재현 방법

```bash
pip install cadquery-ocp numpy matplotlib   # OCP 8.0.1 (Linux 클라우드에서 실행)
cd members/park-jongjin/week04/scripts
python step_tree.py "<양팔로봇 최종버전.stp>" arm.pkl > ../results/step_tree_arm.txt
python step_joint_axes.py "<stp>" > ../results/joint_axes.txt
python step_contacts.py "<stp>" R ; python step_contacts.py "<stp>" L   # 각 ~5분
python step_link_split.py "<stp>" > ../results/link_split_summary.txt  # link_split_parts.csv 생성
python step_link_split_plot.py "<stp>" ../img/20261013_link_split.png
```

## 4. 발견한 문제 · 해결

- (10/13) 노션 커리큘럼 사본 페이지의 「배포 파일」에 STEP 첨부가 없음 → 원본 페이지에서 수령 필요
- (10/13) STEP을 팀장에게 카톡으로 요청 → 수령. **팔 RI85 4개 = 4-DOF**(기획서 6-DOF), 몸통 없음, 전방(±z) 판단 불가 → 링크 분할표 6절로 팀장 확인 요청
- (10/13) FreeCAD 1.0.2 STEP 가져오기: 한글 부품명이 `\X2\C591D314…` 그대로 보임(STEP 유니코드 이스케이프 미변환, CodePage 옵션 무관). 형상·트리는 정상 → 부품명은 OCCT 스크립트 결과(step_tree_arm.txt)로 대조. 가져오기 시 'Ignore instance names' 해제해야 인스턴스 번호 보존
- (10/13) OCP 8.0: `TDF_LabelSequence` 대신 `OCP.collections.Sequence_TDF_Label`, `Bnd_Box.Get()` 대신 CornerMin/Max — 버전 차이
- (10/13) `.gitignore`에 `*.step` · `*.stp` · `*.SLDPRT` · `*.zip` · `bom*` 추가 (원본 커밋 금지 규칙)

## 5. 막힌 점 · 요청 (공유한 날짜)

- 

## 6. 구술 질문 준비 (커리큘럼 4주차)

- Q1. joint origin을 CAD 좌표에서 뽑을 때 부모 링크 프레임 기준으로 바꾸는 절차는
  - 답변 요지:
- Q2. BOM의 정격 토크와 피크 토크 중 effort에 무엇을 쓰고, MuJoCo forcerange · ctrlrange와 어떻게 대응되는가
  - 답변 요지: 초안 → [actuator-spec-request.md §3](../../../b1/docs/icd/actuator-spec-request.md)

## 7. 다음 주 계획 (검증 가능한 단위)

1. 
2. 
3. 

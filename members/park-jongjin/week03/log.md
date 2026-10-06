# week03 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 3 주차 (10/5 ~ 10/8, 10/9 한글날로 제출 목요일) |
| 게이트 | 해당 없음 |
| 제출일 | 2026-10-08 (마감) |
| 저장소 태그 | `week03` |
| 주간 보고서 | [reports/학부생_주간활동보고서_3주차_박종진_22212289_20261008.docx](../reports/) |
| 커리큘럼 | 김이겸, Notion 「B1 2·3·4주차 학습 커리큘럼」 3주차 — **10/6 변경본**: USD · Isaac Sim 실습은 실습 PC 배정 주로 미루고 MuJoCo 심화(MJCF 구성 · 구동 · URDF 로딩)로 대체 |

## 1. 계획 대비 수행

> 실습 · 스크립트 · 문서 작성에 Claude(AI) 보조 사용. 실행은 MuJoCo 3.15.0, 수치는 모두 아래 스크립트 출력(results/)에서 가져옴.

| No. | 계획 (커리큘럼) | 실제 수행 | 완료율 | 산출물 |
|---|---|---|---|---|
| 0 | (월, 변경 전) USD 실습 | 큐브 .usda, 2링크 USD(geom/physics 레이어 · scene reference), URDF 자동 비교 전 항목 통과 → **보고서 부록 A** | 100 % | [usd/](usd/), [scripts/make_two_link_usd.py](scripts/make_two_link_usd.py), [scripts/check_usd_vs_urdf.py](scripts/check_usd_vs_urdf.py) |
| 1 | (화) MJCF 구성 심화 · URDF → MJCF 저장 · include/default 정리 | 2주차 URDF → `mj_saveLastXML`, 팔/scene 분리 + default class 3개. 변환본과 질량 · 관성 · 관절 · damping 일치, 중력토크 3.9731/0.7358 Nm, 궤적 차 1.2e-14 rad | 100 % | [mjcf/two_link_arm.xml](mjcf/two_link_arm.xml), [mjcf/scene.xml](mjcf/scene.xml), [mjcf/two_link_from_urdf.xml](mjcf/two_link_from_urdf.xml), [scripts/tue_urdf_to_mjcf.py](scripts/tue_urdf_to_mjcf.py), [results/tue_compare.txt](results/tue_compare.txt) |
| 2 | (수) position 액추에이터 + kp · kv · armature 계단 응답 3종 | 오버슈트 시뮬/이론 2~4 %p 이내, kp 100 포화, 정상상태 오차 ≈ τg/kp | 100 % | [mjcf/scene_position.xml](mjcf/scene_position.xml), [scripts/wed_step_response.py](scripts/wed_step_response.py), [img/20261006_step_all.png](img/20261006_step_all.png), [results/wed_step_metrics.csv](results/wed_step_metrics.csv) |
| 3 | **(개별)** MuJoCo 로딩 체크리스트 초안 (compiler 옵션 · 베이스 · 자기 충돌 · 액추에이터, 문서 근거) | A~F절, 항목별 MuJoCo 문서 링크 + 실험 E1~E6 | 100 % (초안) | [b1/docs/procedures/mujoco-urdf-loading-checklist.md](../../../b1/docs/procedures/mujoco-urdf-loading-checklist.md), [scripts/thu_loading_experiments.py](scripts/thu_loading_experiments.py), [results/thu_experiments.txt](results/thu_experiments.txt) |
| 4 | (목) CAD → URDF → MJCF 파이프라인 그림 | SVG + PNG | 100 % | [img/pipeline_mujoco.png](img/pipeline_mujoco.png) |
| 5 | **(팀 연계)** 함태훈 URDF ↔ MJCF 대응표 검토 → 체크리스트 G절 | 대응표 대기 | 0 % | |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| URDF/MJCF 기본값이 다른 compiler 속성 | angle(URDF 항상 radian, MJCF degree), fusestatic · discardvisual(URDF true), strippath | [Modeling › URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#curdf) |
| fusestatic=true 결과 | body 5 → 3 (base_link · tool0 소실), 중력토크 동일 3.9731 Nm | thu E3, [compiler-fusestatic](https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler-fusestatic) |
| 부모-자식 충돌 필터 예외 | 부모가 world(용접 포함)면 제외 안 함 → 접촉 2개, exclude 후 0개 | thu E2, [coSelection](https://mujoco.readthedocs.io/en/stable/computation/index.html#coselection) |
| URDF 변환 | effort → actuatorfrcrange, velocity 버려짐, planar → slide·slide·hinge, floating → free | tue, thu E1 |
| angle 생략 함정 | range 1.5708 → ±1.5708° = ±0.0274 rad | thu E4 |
| 베이스 nq/nv (양팔 4관절) | 고정 4/4, free 11/10, 평면 3-DOF 7/7 | thu E5 |
| 평면 베이스 명령 좌표계 | yaw 90° 후 base_x 0.3 m/s → world x 0.25 m (옆걸음) | thu E5, [results/thu_holonomic.csv](results/thu_holonomic.csv) |
| 2차계 근사 | ωn = √(kp/M), ζ = (kv+b)/(2√(kp·M)), M(q=−0.5) = 0.142 kg·m² | wed |
| 오버슈트 시뮬/이론 | kp 20 · 50 · 100: 34.7/35.7, 51.2/53.2, 60.7/64.3 % | wed |
| armature 0 → 0.15 | M 0.142 → 0.292, ωn 18.8 → 13.1 rad/s, 정착 1.1 → 2.1 s | wed, [body-joint-armature](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature) |
| 정상상태 오차 | kp 50: 0.072 rad (이론 τg/kp 0.070) | wed |

## 3. 재현 방법

```bash
pip install mujoco matplotlib          # MuJoCo 3.15.0에서 확인
python members/park-jongjin/week03/scripts/tue_urdf_to_mjcf.py         # 마지막 줄 "결과: 일치"
python members/park-jongjin/week03/scripts/wed_step_response.py
python members/park-jongjin/week03/scripts/thu_loading_experiments.py
python -m mujoco.viewer --mjcf=members/park-jongjin/week03/mjcf/scene.xml
python -m mujoco.viewer --mjcf=members/park-jongjin/week03/mjcf/holonomic_base_two_arms.xml
```

캡처(img/20261006_two_link_mjcf_scene.png, 20261006_holonomic_base_two_arms.png)는 오프스크린 렌더(MUJOCO_GL=osmesa)로 생성.

## 4. 발견한 문제 · 해결

- **2주차 "어깨가 수평에서 안 떨어짐" 원인을 문서 근거로 확정**: MuJoCo 충돌 필터 예외(부모가 world 또는 world에 용접된 body면 부모-자식 접촉을 거르지 않음, Computation › Collision selection 필터 3). fusestatic과는 무관 — on/off 모두 접촉 2개 → `<exclude body1="base_link" body2="link1"/>`로 해결(fusestatic=false여야 이름 참조 가능).
- URDF의 joint `velocity` 한계는 MuJoCo로 옮겨지지 않음 → 체크리스트 A5에 기록, 제어기 · 액추에이터 쪽에서 별도 처리 필요.
- MuJoCo 3.15에서 `mj_fullM` 인자 순서 변경((m, d, dst)) → 스크립트에서 두 버전 모두 처리.
- mjSpec `attach_body`는 원본 spec에서 body를 옮김 → 팔마다 spec을 새로 읽어야 함.

## 5. 막힌 점 · 요청

- armature · forcerange 실제 값에 BOM 구동기 사양(정격 · 피크 토크, 감속비, 로터 관성) 필요 → 보고서 3절로 요청 (2026-10-06)
- 함태훈 URDF ↔ MJCF 대응표 대기 → 체크리스트 G절

## 6. 구술 질문 준비 (커리큘럼 지정)

- Q1. 베이스를 고정할 때와 freejoint로 둘 때의 차이, 홀로노믹 베이스는 MJCF에서 어떻게 표현하는가? → [notes/mujoco-urdf/02-mujoco-advanced.md](../notes/mujoco-urdf/02-mujoco-advanced.md#q1-베이스를-고정할-때와-freejoint로-둘-때의-차이-홀로노믹-베이스는-mjcf에서-어떻게-표현하는가)
- Q2. fusestatic을 켜면 무엇이 사라지고, 언제 꺼야 하는가? → [동 Q2](../notes/mujoco-urdf/02-mujoco-advanced.md#q2-fusestatic을-켜면-무엇이-사라지고-언제-꺼야-하는가)

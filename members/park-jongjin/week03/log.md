# week03 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 3 주차 |
| 게이트 | 해당 없음 |
| 제출일 | YYYY-MM-DD |
| 저장소 태그 | `week03` (제출 시 부여) |
| 주간 보고서 | reports/ |

## 1. 계획 대비 수행

B1 3주차 커리큘럼(USD 개념 + Isaac Sim 개념 + 변환 파이프라인) 기준.

> 2026-09-29 수행, 스크립트 · 초안 작성에 Claude(AI) 보조 사용. **본인 PC에서 재실행 확인 후** 완료율 확정. 5번(팀 연계)은 함태훈 대응표가 나와야 진행 가능.

| No. | 계획 | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | usd-core로 큐브 1개 .usda 생성 → 텍스트로 구조 확인 (Stage · Prim · Attribute · Layer) | Z-up · metersPerUnit=1 · defaultPrim 지정, 텍스트 구조 확인 | 100 % | [usd/cube.usda](usd/cube.usda), [scripts/make_cube_usda.py](scripts/make_cube_usda.py) |
| 2 | 2링크 팔을 Xform 계층으로 USD 표현 → RevoluteJoint 2개 + Drive 속성 추가 | geom 레이어(Xform 계층) + 물리 레이어(sublayer + over: RigidBody · Mass · Collision · RevoluteJoint 2 · DriveAPI · ArticulationRoot) + scene(reference). URDF와 자동 비교 **전 항목 통과** | 100 % | [usd/](usd/), [scripts/make_two_link_usd.py](scripts/make_two_link_usd.py), [scripts/check_usd_vs_urdf.py](scripts/check_usd_vs_urdf.py) |
| 3 | **(개별)** URDF 임포트 체크리스트 초안 — 옵션별 결과 + 문서 근거 | 임포트 전 8 · 옵션 12 · 임포트 후 9개 항목, 옵션마다 5.1 문서/실험 근거 | 100 % (초안) | [b1/docs/procedures/urdf-import-checklist.md](../../../b1/docs/procedures/urdf-import-checklist.md) |
| 4 | CAD → URDF → USD / MJCF 파이프라인 그림 1장 | SVG + PNG | 100 % | [img/pipeline.png](img/pipeline.png), [img/pipeline.svg](img/pipeline.svg) |
| 5 | **(팀 연계)** 함태훈 URDF ↔ USD 대응표 검토 → 체크리스트 반영 | 대기 | 0 % | |

## 2. 핵심 수치 · 근거 (구술 대비)

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| USD Physics 각도 단위 | degree (URDF rad) → ±1.5708 rad = ±90° | [USD Physics](https://openusd.org/release/api/usd_physics_page_front.html), check_usd_vs_urdf.py |
| 기본 밀도 / 우선순위 | 1000 kg/m³, 명시 mass > density | USD Physics |
| Drive 식 | stiffness·(target − p) + damping·(targetVel − v) | USD Physics |
| 임포터 게인 (Natural Frequency) | Kp = m·ωn², Kd = 2·m·ζ·ωn | [Isaac Sim 5.1 URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html) |
| Drive Type 기본 | Acceleration (관성 정규화) / Force | 동 |
| merge fixed joints | 5.1 최신 임포터에서 제거, Isaac Lab은 importer 2.4.31 고정 | [Isaac Lab PR #4000](https://github.com/isaac-sim/IsaacLab/pull/4000) |
| 2링크 USD 검증 | 총질량 3.6 kg, tool0 (0.55, 0, 0.10), 관절 프레임 일치 | check_usd_vs_urdf.py |

## 3. 재현 방법

```bash
cd members/park-jongjin/week03
python scripts/make_cube_usda.py usd
python scripts/make_two_link_usd.py usd
python scripts/check_usd_vs_urdf.py      # 마지막 줄 "결과: 모두 통과"
```

## 4. 발견한 문제 · 해결

- 커리큘럼의 "merge fixed joints 옵션"은 **Isaac Sim 5.1 최신 URDF 임포터에서 제거**됨 (Isaac Lab PR #4000). Isaac Lab `UrdfConverterCfg.merge_fixed_joints` 는 importer 2.4.31 고정으로 유지 → 우리 체크리스트는 "사용하지 않음 + 센서 프레임 링크 보존"으로 정리.
- URDF `limit`(rad)을 USD로 옮길 때 degree 변환 누락이 가장 쉬운 실수 → 자동 비교 스크립트로 검출.

## 5. 막힌 점 · 요청

- Isaac Sim 실제 임포트 결과(링크 계층 · fixed joint 처리 · planar joint 지원)는 실습 PC 필요 → 4주차 확인 항목으로 체크리스트에 남김.

## 6. 구술 질문 준비 (커리큘럼 지정)

- Q1. Isaac Sim에서 Articulation Root 위치가 왜 중요한가?
  - 답변 요지: 고정/부유 베이스를 결정(고정: 월드-베이스 fixed joint 또는 조상, 부유: 루트 링크 또는 조상)하고, 축소 좌표의 기준이 되며(자세·속도는 루트에만 설정), 중첩이 불가하다. 잘못 두면 articulation이 쪼개지거나 관절이 축소 좌표로 풀리지 않는다. → [상세](../notes/usd-isaac/01-usd-basics.md#q1-isaac-sim에서-articulation-root-위치가-왜-중요한가)
- Q2. merge fixed joints 옵션을 켜면 무엇이 사라지는가?
  - 답변 요지: fixed joint로 붙은 자식 링크 prim과 그 좌표계(툴 · 센서 장착 프레임), fixed joint 자체. 질량 · 관성은 부모에 합산돼 물리는 같지만 이름으로 참조 불가 → Isaac Lab 센서 · FrameTransformer 지정 실패. MuJoCo fusestatic 실험에서 body 5 → 3. 5.1 최신 임포터는 이 옵션 제거. → [상세](../notes/usd-isaac/01-usd-basics.md#q2-merge-fixed-joints-옵션을-켜면-무엇이-사라지는가)

## 7. 다음 주 계획

1. (환경 확보 시) 3주차 URDF · USD를 Isaac Sim에 임포트해 Articulation 확인
2. 함태훈 URDF ↔ USD 대응표 검토 → 임포트 체크리스트 v1
3. 실제 임포트 결과로 체크리스트 「확인 필요」 4건 확정

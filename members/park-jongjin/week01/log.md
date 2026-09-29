# week01 기여 로그 — 박종진 (B1)

| 항목 | 내용 |
|---|---|
| 주차 | 제 1 주차 |
| 게이트 | 해당 없음 |
| 제출일 | 2026-09-17 |
| 저장소 태그 | 없음 — 저장소 개설 전 주차. 1주차에 한해 개인 학습 노션 페이지를 기여 로그로 인정받음. 2주차에 본 저장소로 이전 |
| 주간 보고서 | [reports/학부생_주간활동보고서_1주차_박종진_22212289_20260917.docx](../reports/학부생_주간활동보고서_1주차_박종진_22212289_20260917.docx) |

## 1. 계획 대비 수행

| No. | 계획 (전주 합의) | 실제 수행 | 완료율 | 산출물 (파일 경로) |
|---|---|---|---|---|
| 1 | Isaac Sim 5.1 / Isaac Lab 2.3.2 설치 절차 숙지 | 설치 5단계(Ubuntu 24.04·CUDA 12·Miniconda → Isaac Sim 5.1 → IsaacLab v2.3.2 클론 → `_isaac_sim` 심볼릭 링크 → `isaaclab.sh --conda/--install`)와 검증 명령(create_empty.py → Isaac-Ant-v0 headless) 정리. 실습 PC 미확보로 설치 미착수 | 100 % | [notes/isaac-lab/01-install.md](../notes/isaac-lab/01-install.md) |
| 2 | Isaac Lab 기초 데모 구성 및 실행 절차 파악 | list_envs.py, showroom(arms·bipeds·h1_locomotion), zero_agent·random_agent(`--task Isaac-Cartpole-v0 --num_envs 32`) 목적·명령 정리, 첨부 영상 4건으로 동작 확인 | 100 % | [notes/isaac-lab/02-demos.md](../notes/isaac-lab/02-demos.md) |
| 3 | Core Concepts 학습 및 B1 업무 연계점 정리 | Task Design Workflows, Actuators, Sensors 5종, Motion Generators 요약 + B1(URDF/MJCF 모델링·환경 세팅) 적용 지점 | 100 % | [notes/isaac-lab/03-core-concepts.md](../notes/isaac-lab/03-core-concepts.md) |

## 2. 핵심 수치 · 근거

| 수치 / 사실 | 값 | 근거 |
|---|---|---|
| 대상 버전 조합 | Isaac Sim 5.1 · Isaac Lab 2.3.2 · Ubuntu 24.04 · CUDA 12 | 김이겸 Notion「캡스톤디자인(1)」개발환경 구축 |
| 명시적 액추에이터(IdealPD) 토크 | τ = k_p(q_des − q) + k_d(q̇_des − q̇) + τ_ff, ±τ_max 클리핑 | [Isaac Lab Actuators](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/actuators.html) |
| 카메라 대역폭 | 800×600 · 32-bit ≈ 2 MB/장, 60 fps ≈ 120 MB/s | [Isaac Lab Camera](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/camera.html) |
| Tiled Rendering 예 | 64환경 × 84×84 → 672×672 한 장, 동기화 1회 | 동 |
| IMU | 기본 +g 중력 보정 포함 | [Isaac Lab IMU](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/imu.html) |
| Ray Caster | 사전 지정한 정적 mesh만 대상 (Warp) | [Isaac Lab Ray Caster](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/core-concepts/sensors/ray_caster.html) |

## 3. 발견한 문제

- 학습 문서는 Isaac Lab **v2.1.0** 기준, 설치 대상은 **v2.3.2** → ActuatorCfg / SensorCfg 의 2.3.x 변경 인자 확인 필요.

## 4. 막힌 점 · 요청

- 실습용 개발환경(GPU · Ubuntu) 미확보 — 대학원생 "검토 후 회신" 상태.

## 5. 구술 질문

- Q1. `_isaac_sim` 심볼릭 링크는 왜 필요하며, 버전이 어긋나면 어느 단계에서 어떻게 실패하는가?
  - 답변 요지: `isaaclab.sh`는 `IsaacLab/_isaac_sim` 경로를 통해 Isaac Sim의 `python.sh`와 omni 확장(extension)을 찾는다. 링크가 없거나 버전이 맞지 않으면 **임포트 · 확장 로딩 단계**에서 `ModuleNotFoundError` 또는 확장 API 불일치 오류로 실패한다. 검증은 create_empty.py(임포트·앱 기동) → Isaac-Ant-v0 headless(학습 파이프라인) 순.
- Q2. Manager-Based와 Direct 중 B1 URDF 모델 검증 환경에 적합한 것은?
  - 답변 요지: Manager-Based. 관측·보상·행동·이벤트가 매니저와 Cfg 항목으로 분리돼 있어 ArticulationCfg · ActuatorCfg · SensorCfg 만 교체하며 점검할 수 있고, 어느 항목에서 틀어졌는지 격리하기 쉽다.

## 6. 다음 주 계획 (평가자와 합의)

1. 실습 PC 배정 즉시 Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 → create_empty.py, Isaac-Ant-v0 headless 정상 종료 스크린샷
2. Cartpole zero_agent · random_agent(`--num_envs 32`) 직접 실행, 문서 기록과 일치 확인
3. (PC 미배정 시) H1 locomotion env cfg · RewTerm · ObsTerm 정독 → 관측·보상·종료 조건 구조 문서화 + 2.3.x ActuatorCfg/SensorCfg 변경 인자 정리
4. Git 저장소 개인 폴더 생성, 노션 정리본을 md로 옮겨 `week02` 태그로 첫 기여 로그 업로드

# Isaac Lab 기초 데모 (실습 1-2)

> 출처: 김이겸 대학원생 작성 Notion「캡스톤디자인(1)」— Isaac Lab 기초 데모 (학부생 박종진 학습 정리, 1주차).
> 1주차에는 실습 PC 미확보로 **첨부 영상 4건으로 동작만 확인**했고, 직접 실행은 환경 구축 후 진행한다.

| 데모 | 명령 | 목적 | 직접 실행 |
|---|---|---|---|
| 환경 목록 | `./isaaclab.sh -p scripts/environments/list_envs.py` | 등록된 태스크 이름 확인 ([Available Environments](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/environments.html)) | ☐ |
| Showroom — 로봇팔 | `./isaaclab.sh -p scripts/demos/arms.py` | 여러 매니퓰레이터 에셋 로딩 확인 ([Showroom](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/showroom.html)) | ☐ |
| Showroom — 이족 | `./isaaclab.sh -p scripts/demos/bipeds.py` | 이족 로봇 에셋 로딩 확인 | ☐ |
| Zero agent | `./isaaclab.sh -p scripts/environments/zero_agent.py --task Isaac-Cartpole-v0 --num_envs 32` | 행동 0 입력 → 환경 자체 동역학 확인 ([Simple Agents](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/overview/simple_agents.html)) | ☐ |
| Random agent | `./isaaclab.sh -p scripts/environments/random_agent.py --task Isaac-Cartpole-v0 --num_envs 32` | 무작위 행동 → 행동 공간 · 리셋 확인 | ☐ |
| H1 locomotion | `./isaaclab.sh -p scripts/demos/h1_locomotion.py` | 학습된 H1 험지 보행 정책 대화형 추론 | ☐ |

## H1 locomotion 조작키

| 키 | 동작 |
|---|---|
| `UP` | 전진 |
| `LEFT` / `RIGHT` | 좌회전 / 우회전 |
| `DOWN` | 정지 |
| `C` | 3인칭 ↔ 원근 시점 전환 |
| `ESC` | 3인칭 시점 종료 |

## 직접 실행 시 기록할 것 (재현성)

- 실행 PC(GPU 모델 · 드라이버 · CUDA), Isaac Sim / Isaac Lab 버전
- `--num_envs 32` 에서의 FPS, 오류 메시지 원문
- 문서(영상)와 동작이 다르면 차이점
- 스크린샷은 `weekNN/img/YYYYMMDD_<데모명>.png`

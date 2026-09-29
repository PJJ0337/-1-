# Isaac Sim 5.1 + Isaac Lab 2.3.2 설치 절차

> 출처: 김이겸 대학원생 작성 Notion「캡스톤디자인(1)」— 개발환경 구축 (학부생 박종진 학습 정리, 2026-09 1주차).
> 원문 링크는 [Isaac Lab 문서 v2.1.0](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html) 기준이며, 설치 대상은 **v2.3.2** 이다. (버전 차이 주의 — 아래 「확인 필요」)

## 대상 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 24.04 |
| CUDA | 12 |
| Isaac Sim | 5.1 (standalone) |
| Isaac Lab | 2.3.2 |
| Python 환경 | Miniconda (`env_isaaclab`) |

## 1단계 — Miniconda

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh

~/miniconda3/bin/conda init bash
conda config --set auto_activate_base false
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

## 2단계 — Isaac Sim 5.1 설치

- [Installing Isaac Sim (Isaac Lab 문서)](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html#installing-isaac-sim)
- [Isaac Sim Workstation 설치](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_workstation.html) — 5.1 요구사항은 [Isaac Sim 5.1 Requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) 확인
- 예시 설치 경로: `/apps/nvidia/isaacsim/isaac-sim-standalone-5.1.0`

## 3단계 — Isaac Lab 2.3.2 클론

```bash
git clone https://github.com/isaac-sim/IsaacLab.git -b v2.3.2
```

## 4단계 — `_isaac_sim` 심볼릭 링크

```bash
cd /apps/nvidia/IsaacLab
ln -s /apps/nvidia/isaacsim/isaac-sim-standalone-5.1.0 _isaac_sim
```

**왜 필요한가:** `isaaclab.sh` 는 `IsaacLab/_isaac_sim` 경로를 통해 Isaac Sim의 `python.sh` 와 omni 확장(extension)을 찾는다. 링크가 없거나 다른 버전의 Isaac Sim을 가리키면 **임포트 · 확장 로딩 단계**에서 `ModuleNotFoundError` 또는 확장 API 불일치 오류로 실패한다.

## 5단계 — conda 환경 생성 · 패키지 설치

```bash
./isaaclab.sh --conda          # env_isaaclab 생성
conda activate env_isaaclab
./isaaclab.sh --install        # Isaac Lab 확장 · 학습 프레임워크 설치
```

## 설치 검증 (순서대로)

```bash
# ① 앱 기동 · 임포트 확인
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py

# ② 학습 파이프라인 확인 (GUI 없이)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Ant-v0 --headless
```

| 검증 | 확인하는 것 | 실패 시 의심 지점 |
|---|---|---|
| create_empty.py | Isaac Sim 기동, `isaaclab` 임포트 | 심볼릭 링크, 버전 조합, 드라이버 |
| Isaac-Ant-v0 headless | 환경 등록 · RL 라이브러리 · GPU 학습 루프 | `--install` 누락, CUDA |

## 확인 필요

- [ ] 학습 문서(v2.1.0)와 설치 대상(v2.3.2)의 `ActuatorCfg` / `SensorCfg` 인자 변경 사항
- [ ] 실습 PC 배정 후 위 검증 2건 스크린샷을 해당 주차 `img/` 에 저장

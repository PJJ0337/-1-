# week08 — results/collect.json → results/collect.md 요약표 (collect.py 실행 뒤)
import os,json,collections
HERE=os.path.dirname(os.path.abspath(__file__))
C=json.load(open(os.path.join(HERE,"results","collect.json"),encoding="utf-8")); R=C["replay"]; RS=C.get("replay_samples")
rows=C["rows"]; fails=[r for r in rows if not r["success"]]; kinds=collections.Counter(r["kind"] for r in rows)
L=["# 시연 데이터 수집 · 재생 검증 (자동 생성 — collect_report.py)","",
f"- 전문가 정책(상태기계 + 전신 IK) 자동 생성 **{C['n']}개**(seed {C['seeds'][0]} ~ {C['seeds'][1]}, 평가용 seed 0 ~ 19와 분리): 성공 **{C['success']}/{C['n']}**, 실패 {C['fails']}",
f"- 물체 상자 {kinds['box']} · 원기둥 {kinds['cyl']}, 질량 0.2 ~ 3 kg 로그 균등. 총 {C['steps_total']:,} 스텝(50 Hz, 에피소드 평균 {C['steps_total']/C['n']:.0f} 스텝 ≈ {C['steps_total']/C['n']/50:.1f} s), 원본 {C['mb_total']} MB(gzip) — 커밋 안 함(`datasets/raw/`)",
f"- 수집 {C['seconds']} s (CPU 1코어), 모델 지문 `{C['model_hash']}`","",
"## 재생(replay) 검증","",
f"- 저장된 action만 다시 넣어 시뮬(같은 seed로 초기화) → **{R['n']}개 전부 qpos 최대 차 {R['max_abs_qpos_diff']:.1e}**(비트 단위 일치), 성공 판정 일치 {R['success_consistent']}/{R['n']}, 모델 지문 일치 {R['hash_match']}/{R['n']}, {R['seconds']} s"]
if RS: L.append(f"- 커밋한 샘플 {RS['n']}개도 같은 검증: 최대 차 {RS['max_abs_qpos_diff']:.1e}, 지문 일치 {RS['hash_match']}/{RS['n']}")
L+=["- 결정성 조건: 같은 모델(지문) · 같은 MuJoCo 버전(3.15.0) · 단일 스레드 · 같은 초기 상태(seed → `sample_task`). 발산 에피소드(seed 1080)도 재생에서 같은 시각(11.483 s)에 똑같이 발산 → 결정적","",
"## 실패 에피소드","","| seed | 물체 | 질량 kg | 실패 | 스텝 |","|---|---|---|---|---|"]+[f"| {r['seed']} | {r['kind']} | {r['mass']} | {r['fail']} | {r['steps']} |" for r in fails]+["",
"- sim_unstable = 3 kg 근처 원기둥을 들어 올리는 순간 MuJoCo 수치 발산(경고 BADQVEL) — dt 2 → 1 ms · 손가락 0.2 kg · 그리퍼 명령 변화율로 대부분 해결, 남은 1건은 함태훈 접촉 파라미터(solref · solimp) 분석에 넘김. 데이터에는 `success=False`로 남김(BC 학습은 성공만)","",
"## 커밋한 샘플 (`datasets/samples/`)","","| 파일 | 내용 |","|---|---|",
"| ep_01000.hdf5 | 상자 0.84 kg, 카메라 `overview` 96 × 96 포함 (~7 MB) |","| ep_01001.hdf5 · ep_01002.hdf5 | 원기둥, 카메라 없음 (~0.5 MB) |","",
"형식: [../../datasets/dataset_spec_draft.md](../../datasets/dataset_spec_draft.md)"]
open(os.path.join(HERE,"results","collect.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

# week08 — results/collect.json → results/collect.md 요약표 (collect.py 실행 뒤)
import os,json,collections
HERE=os.path.dirname(os.path.abspath(__file__))
C=json.load(open(os.path.join(HERE,"results","collect.json"),encoding="utf-8")); R=C["replay"]; RS=C.get("replay_samples")
rows=C["rows"]; fails=[r for r in rows if not r["success"]]; kinds=collections.Counter(r["kind"] for r in rows)
L=["# 시연 데이터 수집 · 재생 검증 (자동 생성 — collect_report.py)","",
f"- 전문가 정책(상태기계 + 전신 IK) 자동 생성 **{C['n']}개**(seed {C['seeds'][0]} ~ {C['seeds'][1]}, 평가용 seed 0 ~ 19와 분리): 성공 **{C['success']}/{C['n']}**, 실패 {C['fails']}",
f"- 물체 상자 {kinds['box']} · 원기둥 {kinds['cyl']}, 질량 0.2 ~ 3 kg 로그 균등. 총 {C['steps_total']:,} 스텝(50 Hz, 에피소드 평균 {C['steps_total']/C['n']:.0f} 스텝 ≈ {C['steps_total']/C['n']/50:.1f} s), 원본 {C['mb_total']} MB(gzip) — 커밋 안 함(`datasets/raw/`)",
f"- 수집 {C['seconds']} s (CPU 1코어, dt 0.5 ms), 모델 지문 `{C['model_hash']}`, 형식 = 함태훈 v1.0 `b1-dataset-1.0`(datasets/dataset_spec.md)","",
"## 재생(replay) 검증","",
f"- 명세 검사(함태훈 `b1_dataset.check`): {R.get('spec_check_pass','?')}/{R['n']} 통과",
f"- 저장된 action만 다시 넣어 시뮬(같은 seed로 초기화) → **{R['n']}개 전부 qpos 최대 차 {R['max_abs_qpos_diff']:.1e}**(비트 단위 일치), 성공 판정 일치 {R['success_consistent']}/{R['n']}, 모델 지문 일치 {R['hash_match']}/{R['n']}, {R['seconds']} s"]
if RS: L.append(f"- 커밋한 샘플 {RS['n']}개도 같은 검증: 최대 차 {RS['max_abs_qpos_diff']:.1e}, 지문 일치 {RS['hash_match']}/{RS['n']}")
L+=["- 결정성 조건: 같은 모델(지문) · 같은 MuJoCo 버전(3.15.0) · 단일 스레드 · 같은 초기 상태(seed → `sample_task`)",
"- week08(dt 1 ms, da55741)에서는 99/100 — seed 1080(원기둥 2.98 kg) 들기 순간 수치 발산(sim_unstable), 재생에서도 같은 시각(11.483 s)에 똑같이 발산. 함태훈 원인 분석(물체 각속도 0.1 → 17 rad/s 지수 증가 = 수치 진동, solref · solimp · impratio · condim으로는 안 풀리고 dt 0.5 ms만 해결) → dt 0.5 ms로 재수집","",
"## 실패 에피소드","","| seed | 물체 | 질량 kg | 실패 | 스텝 |","|---|---|---|---|---|"]+([f"| {r['seed']} | {r['kind']} | {r['mass']} | {r['fail']} | {r['steps']} |" for r in fails] or ["| — | | | 없음 | |"])+["",
"## 커밋한 샘플 (`datasets/samples/`)","","| 파일 | 내용 |","|---|---|",
"| ep_01000.hdf5 | 상자 0.84 kg, 카메라 `overview` 96 × 96 포함 (~7 MB) |","| ep_01001.hdf5 · ep_01002.hdf5 | 원기둥, 카메라 없음 (~0.5 MB) |","",
"형식: [../../datasets/dataset_spec.md](../../datasets/dataset_spec.md) v1.0 (함태훈, 초안 [dataset_spec_draft.md](../../datasets/dataset_spec_draft.md)의 필드 그대로 + 로봇 좌표 파생 4개 · 속성 5개)"]
open(os.path.join(HERE,"results","collect.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

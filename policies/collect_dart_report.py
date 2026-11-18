# week09 박종진 — DART 수집 요약: datasets/raw_dart/*.hdf5 + 수집 로그(저장 안 된 sim_unstable) → results/collect_dart.{json,md}
import os,sys,json,glob,collections
import h5py,numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
F=sorted(glob.glob(os.path.join(ROOT,"datasets","raw_dart","ep_*.hdf5"))); rows=[]
for f in F:
    with h5py.File(f) as h:
        a=h["action"][:]; l=h["action_label"][:]
        rows.append(dict(seed=int(h.attrs["seed"]),kind=str(h.attrs["kind"]),success=bool(h.attrs["success"]),fail=str(h.attrs["fail"]) or None,steps=len(a),
                         noise_base_rms=float(np.sqrt(np.mean((a[:,:3]-l[:,:3])**2))),noise_arm_rms_deg=float(np.degrees(np.sqrt(np.mean((a[:,3:7]-l[:,3:7])**2))))))
seeds=set(r["seed"] for r in rows); n_try=int(sys.argv[1]) if len(sys.argv)>1 else 300; s0=3000
unsaved=[s for s in range(s0,s0+n_try) if s not in seeds]
ok=[r for r in rows if r["success"]]
C=dict(seeds=[s0,s0+n_try-1],tried=n_try,saved=len(rows),unsaved_sim_unstable=unsaved,success=len(ok),fails=dict(collections.Counter(r["fail"] for r in rows if not r["success"])),
       frames_success=sum(r["steps"] for r in ok),frames_all=sum(r["steps"] for r in rows),
       noise_rms=dict(base=round(float(np.mean([r["noise_base_rms"] for r in rows])),4),arm_deg=round(float(np.mean([r["noise_arm_rms_deg"] for r in rows])),3)),rows=rows)
json.dump(C,open(os.path.join(HERE,"results","collect_dart.json"),"w"),indent=1,ensure_ascii=False)
L=["# DART 노이즈 시연 수집 요약 (자동 생성 — `collect_dart_report.py`, week09 박종진)","",
   f"seed {C['seeds'][0]} ~ {C['seeds'][1]} {n_try}회 → 저장 {C['saved']} (수치 발산으로 명세 check 실패 → 저장 안 함 {len(unsaved)}: {unsaved}), **성공 {C['success']}** → 학습에 씀(성공 프레임 {C['frames_success']:,})",
   f"실제 넣은 명령 − 전문가 명령 RMS: 베이스 {C['noise_rms']['base']} (m/s · rad/s), 팔 {C['noise_rms']['arm_deg']}°","",
   "| 실패 분류 | 개수 |","|---|---|"]+[f"| {k} | {v} |" for k,v in sorted(C["fails"].items(),key=lambda kv:-kv[1])]
L+=["","- 노이즈: OU(상관 시간 0.3 s), 정상상태 σ 베이스 0.05 m/s · 0.08 rad/s, 팔 0.01 rad, 그리퍼 0, 정밀 단계(ALIGN ~ RELEASE) 30 % — `collect_dart.py`",
    "- 처음 σ 팔 0.03 rad에서는 6/6 실패(끝점 ~15 mm > 정렬 허용 6 mm) → 0.01 rad + 정밀 단계 축소",
    "- 실패 대부분이 approach_timeout · align_fail(허용 오차 안으로 못 들어옴) = 노이즈가 계속 들어가서 — 학습에는 성공 에피소드만"]
open(os.path.join(HERE,"results","collect_dart.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

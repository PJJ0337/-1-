# week09 박종진 — 관절 출력 vs 끝점 출력 BC: 같은 검증 시연(seed 1090 ~ 1099)에서 open-loop 오차를 같은 단위로 비교
#   팔 목표를 끝점 공간으로: 관절 출력 = (현재 팔 + 예측 잔차) → FK, 끝점 출력 = 현재 끝점 + 예측 차 (+ 예측 z축)
#   라벨 = 시연 관절 목표 → FK. 위치 오차 [mm] · 그리퍼 축 오차 [°], 베이스 속도 오차 [mm/s · °/s], 그리퍼 [mm]
#   끝점 출력은 롤아웃 때 IK(직전 해 시작)를 한 번 더 거침 → IK 잔차도 기록
#   실행: python policies/offline_compare.py joint ee joint_dart ee_dart  → results/offline_compare.{json,md}
import os,sys,json
import numpy as np,h5py,torch
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,HERE)
import bc_common as B, pick_env as P
from train_bc import Policy
torch.set_num_threads(1); m=P.build(cam=True); kin=B.Kin(m)
tags=sys.argv[1:] or ["joint","ee"]; pols={t:Policy(os.path.join(HERE,"ckpt",f"bc_{t}.pt")) for t in tags}
files=[os.path.join(ROOT,"datasets","raw",f"ep_{s:05d}.hdf5") for s in range(1090,1100)]
acc={t:dict(pos=[],ax=[],vb=[],wz=[],grip=[],ikres=[]) for t in tags}
for f in files:
    with h5py.File(f) as h:
        o=h["obs"]; Q=o["qpos"][:]; V=o["qvel"][:]; EP=o["ee_pos"][:]; EQ=o["ee_quat"][:]; OP=o["obj_pos"][:]; OQ=o["obj_quat"][:]; G=o["gripper_width"][:]; A=h["action"][:]; kind=str(h.attrs["kind"])
    for t in range(0,len(A),2):
        x=B.features(Q[t],V[t],EP[t],EQ[t],OP[t],OQ[t],G[t],kind); pl,zl=kin.ee_target_base(Q[t],A[t,3:7])
        R=B.Rz(Q[t,2]); vbl=R[:2,:2].T@A[t,0:2]; tb=np.array([Q[t,0],Q[t,1],0.0]); ee_b=R.T@(EP[t]-tb)
        for k,pol in pols.items():
            y=pol.act(x)[:{"joint":8,"ee":10}[pol.out]]
            if pol.out=="joint": pp,zp=kin.ee_target_base(Q[t],Q[t,3:7]+y[3:7])
            else:
                pp,zp=ee_b+y[3:6],y[6:9]/max(np.linalg.norm(y[6:9]),1e-9)
                q,res=kin.ik(Q[t].copy(),Q[t,3:7],pp,zp); acc[k]["ikres"].append(res*1e3)
            a_=acc[k]; a_["pos"].append(np.linalg.norm(pp-pl)*1e3); a_["ax"].append(np.degrees(np.arccos(np.clip(zp@zl/np.linalg.norm(zl),-1,1))))
            a_["vb"].append(np.linalg.norm(y[0:2]-vbl)*1e3); a_["wz"].append(abs(np.degrees(y[2]-A[t,2]))); a_["grip"].append(abs(y[-1]-A[t,7])*1e3)
out={k:{q:dict(mean=round(float(np.mean(v)),3),p95=round(float(np.percentile(v,95)),3),max=round(float(np.max(v)),3)) for q,v in a.items() if v} for k,a in acc.items()}
json.dump(dict(files=[os.path.basename(f) for f in files],every=2,result=out),open(os.path.join(HERE,"results","offline_compare.json"),"w"),indent=1,ensure_ascii=False)
L=["# 관절 출력 vs 끝점 출력 — 검증 시연 open-loop 오차 (자동 생성 — `offline_compare.py`, week09 박종진)","",
   "검증 시연 10개(seed 1090 ~ 1099, 학습에 안 씀) 2주기마다. 팔 목표는 둘 다 끝점 공간(로봇 좌표)으로 바꿔 비교 · 라벨 = 시연 관절 목표의 FK","",
   "| 모델 | 끝점 목표 위치 평균 / 95 % / 최대 [mm] | 그리퍼 축 평균 / 95 % [°] | 베이스 속도 평균 [mm/s] | wz 평균 [°/s] | 그리퍼 평균 [mm] | IK 잔차 평균 / 최대 [mm] |","|---|---|---|---|---|---|---|"]
for k,r in out.items():
    ik=f"{r['ikres']['mean']} / {r['ikres']['max']}" if "ikres" in r else "— (IK 없음)"
    L.append(f"| {k} | {r['pos']['mean']} / {r['pos']['p95']} / {r['pos']['max']} | {r['ax']['mean']} / {r['ax']['p95']} | {r['vb']['mean']} | {r['wz']['mean']} | {r['grip']['mean']} | {ik} |")
open(os.path.join(HERE,"results","offline_compare.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))

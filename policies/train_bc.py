# week09 박종진 — BC 학습 (PyTorch CPU): 상태 기반 MLP obs(39) → action(8 | 10)
#   데이터 = 8주차 수집 100 에피소드(datasets/raw, seed 1000 ~ 1099, 커밋 안 함) → 에피소드 단위로 학습 90 / 검증 10 분리(같은 에피소드의 이웃 프레임이 양쪽에 섞이지 않게)
#   정규화 = 학습 세트 평균 · 표준편차(입력 · 출력 둘 다, 체크포인트에 같이 저장), 손실 = 정규화 공간 MSE
#   --noise σ : 입력 노이즈 증강(정규화 공간 가우시안) — 공변량 이동 완화용 비교 조건(DART 단순형: 라벨은 그대로)
#   --data 폴더1,폴더2 : 첫 폴더 = 깨끗한 시연(검증 10개를 여기서 뗌), 나머지 = DART 노이즈 시연(collect_dart.py, 라벨 = action_label)
#   실행: python policies/train_bc.py --out joint            (~2 분, CPU)
#         python policies/train_bc.py --out joint --data datasets/raw,datasets/raw_dart --tag joint_dart
#         python policies/train_bc.py --out ee
#         python policies/train_bc.py --out joint --noise 0.1 --tag joint_n01
#   출력: policies/ckpt/bc_<tag>.pt (~0.6 MB, 커밋), policies/results/train_<tag>.json (손실 곡선)
import os,sys,json,time,argparse
import numpy as np,torch,torch.nn as nn
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE); sys.path.insert(0,HERE)
import bc_common as B

class MLP(nn.Module):
    def __init__(s,ni,no,h=256,n=3):
        super().__init__(); L=[]; d=ni
        for _ in range(n): L+=[nn.Linear(d,h),nn.ReLU()]; d=h
        L.append(nn.Linear(d,no)); s.net=nn.Sequential(*L)
    def forward(s,x): return s.net(x)

class Policy:
    """체크포인트 → 추론기(정규화 포함). act(feat) → 원래 단위 출력"""
    def __init__(s,path):
        ck=torch.load(path,map_location="cpu",weights_only=False); s.cfg=ck["cfg"]
        s.net=MLP(len(ck["xm"]),len(ck["ym"]),ck["cfg"]["hidden"],ck["cfg"]["layers"]); s.net.load_state_dict(ck["state"]); s.net.eval()
        s.xm,s.xs,s.ym,s.ys=[np.asarray(ck[k],np.float32) for k in ("xm","xs","ym","ys")]; s.out=ck["cfg"]["out"]; s.hist=ck["cfg"].get("hist",""); s.chunk=ck["cfg"].get("chunk",1)
    @torch.no_grad()
    def act(s,x):
        z=torch.from_numpy(((x-s.xm)/s.xs).astype(np.float32))[None]; return s.net(z)[0].numpy()*s.ys+s.ym

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--out",default="joint",choices=["joint","ee"]); ap.add_argument("--data",default=os.path.join(ROOT,"datasets","raw"))
    ap.add_argument("--epochs",type=int,default=40); ap.add_argument("--bs",type=int,default=512); ap.add_argument("--lr",type=float,default=1e-3)
    ap.add_argument("--hidden",type=int,default=256); ap.add_argument("--layers",type=int,default=3); ap.add_argument("--noise",type=float,default=0.0); ap.add_argument("--prev_noise",type=float,default=0.0,help="직전 행동 입력에만 넣는 노이즈(정규화 공간) — copycat(직전 행동 베끼기) 완화")
    ap.add_argument("--hist",default="",help='"" | prev | time | prev,time'); ap.add_argument("--chunk",type=int,default=1,help="액션 청킹 K (joint 출력만): 앞으로 K주기 명령을 한 번에 예측, 롤아웃은 시간 앙상블"); ap.add_argument("--n_val",type=int,default=10); ap.add_argument("--seed",type=int,default=0); ap.add_argument("--tag")
    a=ap.parse_args(); tag=a.tag or a.out; torch.manual_seed(a.seed); np.random.seed(a.seed); torch.set_num_threads(max(1,os.cpu_count()))
    t0=time.time(); dirs=a.data.split(","); eps=B.load_dir(dirs[0],a.out,chunk=a.chunk)       # 첫 폴더 = 깨끗한 시연(8주차 100개)
    extra=[e for dd in dirs[1:] for e in B.load_dir(dd,a.out,chunk=a.chunk)]                  # 추가 폴더 = DART 노이즈 시연(성공만)
    t_load=time.time()-t0
    eps=[(B.add_hist(X,Y,a.hist,0.02),Y,sd) for X,Y,sd in eps]; extra=[(B.add_hist(X,Y,a.hist,0.02),Y,sd) for X,Y,sd in extra]
    tr,va=eps[:-a.n_val]+extra,eps[-a.n_val:]                   # 검증 = 깨끗한 시연 마지막 10개(seed 1090 ~ 1099), 모든 설정에서 같은 검증 세트
    Xtr=np.concatenate([e[0] for e in tr]); Ytr=np.concatenate([e[1] for e in tr]); Xva=np.concatenate([e[0] for e in va]); Yva=np.concatenate([e[1] for e in va])
    xm,xs=Xtr.mean(0),Xtr.std(0)+1e-6; ym,ys=Ytr.mean(0),Ytr.std(0)+1e-6
    xs[xs<1e-3]=1.0                                             # 거의 상수인 특징(예: 테이블 높이 쪽 goal_z)은 나누지 않음
    f=lambda X,m_,s_: torch.from_numpy(((X-m_)/s_).astype(np.float32))
    xt,yt,xv,yv=f(Xtr,xm,xs),f(Ytr,ym,ys),f(Xva,xm,xs),f(Yva,ym,ys)
    nf,ny=len(B.FEAT_NAMES),yt.shape[1]                         # 직전 행동 열 = 특징 39 뒤 ny개
    net=MLP(xt.shape[1],yt.shape[1],a.hidden,a.layers); opt=torch.optim.AdamW(net.parameters(),lr=a.lr,weight_decay=1e-4)
    steps=a.epochs*int(np.ceil(len(xt)/a.bs)); sch=torch.optim.lr_scheduler.CosineAnnealingLR(opt,steps)
    hist=[]; t1=time.time()
    for ep in range(a.epochs):
        net.train(); perm=torch.randperm(len(xt)); tl=0.0
        for i in range(0,len(xt),a.bs):
            idx=perm[i:i+a.bs]; xb=xt[idx]
            if a.noise>0: xb=xb+a.noise*torch.randn_like(xb)
            if a.prev_noise>0 and "prev" in a.hist:
                xb=xb.clone(); xb[:,nf:nf+ny]+=a.prev_noise*torch.randn(len(xb),ny)
            loss=((net(xb)-yt[idx])**2).mean(); opt.zero_grad(); loss.backward(); opt.step(); sch.step(); tl+=loss.item()*len(idx)
        net.eval()
        with torch.no_grad(): vl=((net(xv)-yv)**2).mean(0)
        hist.append(dict(epoch=ep+1,train=tl/len(xt),val=float(vl.mean()),val_per_dim=[round(float(v),4) for v in vl]))
        if (ep+1)%5==0 or ep==0: print(f"[{tag}] ep {ep+1:3d} train {hist[-1]['train']:.4f} val {hist[-1]['val']:.4f}",flush=True)
    cfg=dict(out=a.out,hist=a.hist,chunk=a.chunk,prev_noise=a.prev_noise,n_extra_eps=len(extra),hidden=a.hidden,layers=a.layers,epochs=a.epochs,bs=a.bs,lr=a.lr,noise=a.noise,seed=a.seed,
             feat_names=B.FEAT_NAMES,act_names=[f"{n}@{k}" for k in range(a.chunk) for n in B.ACT_NAMES[a.out]] if a.chunk>1 else B.ACT_NAMES[a.out],train_seeds=[e[2] for e in tr],val_seeds=[e[2] for e in va])
    os.makedirs(os.path.join(HERE,"ckpt"),exist_ok=True); os.makedirs(os.path.join(HERE,"results"),exist_ok=True)
    torch.save(dict(cfg=cfg,state=net.state_dict(),xm=xm,xs=xs,ym=ym,ys=ys),os.path.join(HERE,"ckpt",f"bc_{tag}.pt"))
    json.dump(dict(cfg=cfg,n_train=len(xt),n_val=len(xv),n_params=sum(p.numel() for p in net.parameters()),seconds_load=round(t_load,1),
                   seconds_train=round(time.time()-t1,1),torch=torch.__version__,hist=hist),open(os.path.join(HERE,"results",f"train_{tag}.json"),"w"),indent=1,ensure_ascii=False)
    print(f"[{tag}] 학습 {len(xt)} · 검증 {len(xv)} 프레임, 최종 val {hist[-1]['val']:.4f}, {time.time()-t1:.0f} s")

if __name__=="__main__": main()

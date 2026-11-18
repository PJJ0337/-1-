# week09 박종진 — 함태훈 랜덤화(ed160f2 sim/w09/bc_eval_pjj.py)로 B1 BC를 평가한 결과를 B1 쪽에서 다시 돌려 확인(「없음 · 동시 ×1」, 전문가 · BC 청킹 25)
#   함태훈 저장소를 옆에 clone: git clone https://github.com/taehoonham0118-cell/-1-.git ../taehoon && git -C ../taehoon checkout ed160f2
#   실행: TAEHOON=../taehoon python policies/eval_taehoon_repro.py      (~8 분, 2 프로세스) → results/eval_taehoon_repro.json
import sys,os,json,collections
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
TH=os.path.abspath(os.environ.get("TAEHOON",os.path.join(ROOT,"..","taehoon")))
sys.argv=[sys.argv[0]]; os.environ["PJJ"]=ROOT; sys.path.insert(0,os.path.join(TH,"sim","w09"))
import bc_eval_pjj as X          # 함태훈 감싸기(물체 질량 · 측정 편향 · 지연) + randomize.make_model(테이블 높이 · 마찰 · 질량 · 구동기)
from multiprocessing import Pool
if __name__=="__main__":
    R=X.R; nom=R.nominal(); seeds=range(2000,2050)
    jobs=[(w,lab,f(s),s) for w in ("expert","joint_dart_c25") for lab,f in (("없음",lambda s:nom),("동시 ×1",lambda s:R.sample_joint(s,1.0))) for s in seeds]
    with Pool(2) as pool: rows=pool.map(X.one,jobs)
    G=collections.defaultdict(list)
    for r in rows: G[(r["who"],r["tag"])].append(r)
    S={f"{a}|{b}":dict(success=sum(x["success"] for x in g),n=len(g),fails=dict(collections.Counter(x["fail"] for x in g if not x["success"]))) for (a,b),g in G.items()}
    keep=[{k:r.get(k) for k in ("who","tag","seed","kind","mass","success","fail","goal_err_mm")} for r in rows]
    json.dump(dict(taehoon_commit="ed160f2",summary=S,rows=keep),open(os.path.join(HERE,"results","eval_taehoon_repro.json"),"w"),ensure_ascii=False,indent=1,default=str)
    print(json.dumps(S,ensure_ascii=False,indent=1))

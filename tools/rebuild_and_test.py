# 모델 갱신 파이프라인 (week10, 박종진 — 함태훈 df1e64d tools/rebuild_and_test.py 를 이 저장소 구조로 확장)
#   config/params.yaml → build_v1.py(B1_A2_CASE = shoulder_case) 로 MJCF · URDF 재생성 → pytest tests/ → docs/regression_report.md 자동 생성
#   원본 저장소는 건드리지 않음: b1/ · scenarios/ · config/ 를 build/ 로 복사해 그 안에서 재빌드 · 테스트 (생성된 MJCF 를 손으로 고치지 않는다)
#   A팀 변경(질량 · 관절 원점 · 구동기)이 들어오면
#     ① b1/models/v1/inputs/*.json 또는 config/params.yaml 의 값 · 출처 · 날짜 수정  ② 이 스크립트 한 번 실행
#     ③ 빨간 줄(실패한 테스트) = 바뀐 숫자 → 의도한 변경이면 params.yaml regression 값을 갱신하고 log 에 이유를 적음, 아니면 입력을 되돌림
#     ④ 결과가 맞으면 저장소 원본에도 build_v1.py 를 실행(같은 md5 확인)하고 커밋
#   실행: python tools/rebuild_and_test.py [--params config/params.yaml] [--fast(-m "not slow")] [--out build]
#   출력: docs/regression_report.md (기본) · build/junit.xml
import os,sys,shutil,subprocess,argparse,hashlib,datetime,time,xml.etree.ElementTree as ET
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0,os.path.join(ROOT,"config")); import params as PRM
ap=argparse.ArgumentParser(); ap.add_argument("--params",default=os.path.join(ROOT,"config","params.yaml")); ap.add_argument("--out",default=os.path.join(ROOT,"build"))
ap.add_argument("--fast",action="store_true"); ap.add_argument("--report",default=os.path.join(ROOT,"docs","regression_report.md")); a=ap.parse_args()
par=PRM.load(a.params); case=par["model"]["shoulder_case"]; B=os.path.abspath(a.out)
if os.path.exists(B): shutil.rmtree(B)
for sub in ("b1","scenarios","config"): shutil.copytree(os.path.join(ROOT,sub),os.path.join(B,sub),ignore=shutil.ignore_patterns("__pycache__","*.hdf5","dart*"))
V1=os.path.join(B,"b1","models","v1"); env=dict(os.environ,B1_A2_CASE=case,PYTHONIOENCODING="utf-8")
t0=time.time(); r=subprocess.run([sys.executable,os.path.join(V1,"scripts","build_v1.py")],env=env,capture_output=True,text=True,encoding="utf-8")
if r.returncode: print(r.stderr[-2000:]); sys.exit("build_v1 실패")
tb=time.time()-t0
h=lambda p: hashlib.md5(open(p,"rb").read()).hexdigest()[:8]
md5_new=h(os.path.join(V1,"mjcf","b1.xml")); md5_src=h(os.path.join(ROOT,"b1","models","v1","mjcf","b1.xml"))
junit=os.path.join(B,"junit.xml")
env2=dict(env,B1_ROOT=B,B1_V1=V1,PJJ=B,B1_PARAMS=os.path.abspath(a.params))
cmd=[sys.executable,"-m","pytest",os.path.join(ROOT,"tests"),"-q","-p","no:cacheprovider",f"--junitxml={junit}","-o","junit_family=xunit1"]+(["-m","not slow"] if a.fast else [])
t1=time.time(); t=subprocess.run(cmd,env=env2,capture_output=True,text=True,encoding="utf-8",cwd=ROOT); tt=time.time()-t1
print(t.stdout[-3000:])
X=ET.parse(junit).getroot(); X=X if X.tag=="testsuite" else X.find("testsuite")
rows=[]
for c in X.iter("testcase"):
    st="PASS"; msg=""
    for k in ("failure","error"):
        e=c.find(k)
        if e is not None: st="FAIL" if k=="failure" else "ERROR"; msg=(e.get("message") or "").splitlines()[0][:120]
    if c.find("skipped") is not None: st="SKIP"
    props={p.get("name"):p.get("value") for p in c.iter("property")}
    rows.append(dict(file=c.get("classname","").split(".")[-1],name=c.get("name"),status=st,time=float(c.get("time") or 0),props=props,msg=msg))
n=len(rows); npass=sum(r["status"]=="PASS" for r in rows); nfail=n-npass-sum(r["status"]=="SKIP" for r in rows)
L=["# 회귀 테스트 보고서 (자동 생성 — tools/rebuild_and_test.py)","",
   f"- 일시 {datetime.datetime.now():%Y-%m-%d %H:%M} · params `{os.path.relpath(os.path.abspath(a.params),ROOT)}` (updated {par.get('updated')}) · shoulder_case **{case}** · payload_limit {par['limits']['payload_limit']['enable']} · reach_limit {par['limits']['reach_limit']['enable']} · 짐 {par['payload']['target_kg']} kg",
   f"- 재빌드: build_v1.py (B1_A2_CASE={case}) {tb:.1f} s · b1.xml md5 `{md5_new}` — 저장소 원본 `{md5_src}` 와 **{'같음' if md5_new==md5_src else '다름(입력 · case 변경)'}**",
   f"- pytest {'(빠른 것만, -m \"not slow\")' if a.fast else '(전체)'}: **{npass}/{n} 통과**{f' · 실패 {nfail}' if nfail else ''} · {tt:.0f} s",
   "- 통과 기준 · 허용 오차의 이유는 각 테스트 파일 머리 주석(tests/test_*.py)과 config/params.yaml regression 에 있음","",
   "| 파일 | 테스트 | 결과 | 시간 s | 기록 숫자 |","|---|---|---|---|---|"]
for r in rows:
    pr=" · ".join(f"{k} {v}" for k,v in r["props"].items())
    L.append(f"| {r['file']} | {r['name']} | {'✅' if r['status']=='PASS' else ('⏭' if r['status']=='SKIP' else '❌ '+r['msg'])} | {r['time']:.1f} | {pr} |")
L+=["","담당: test_validation5 · test_torque = 함태훈(df1e64d 사본) / test_ik · test_scenario = 박종진"]
os.makedirs(os.path.dirname(a.report),exist_ok=True); open(a.report,"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
sys.exit(t.returncode)

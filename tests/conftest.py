# pytest 공통 (week10) — 박종진 저장소. 함태훈 df1e64d tests/conftest.py 와 같은 fixture(par · scene · arm) + 집기 scene(pick)
#   모델 = B1_V1(재빌드 폴더) 또는 이 저장소 b1/models/v1, 설정 = config/params.yaml (B1_PARAMS 로 바꿀 수 있음)
#   실행: python tools/rebuild_and_test.py  (params → build_v1 → pytest → docs/regression_report.md)   또는  pytest tests/ -q
#   느린 테스트(집기 20회 · 방안 비교)는 -m "not slow" 로 뺄 수 있음
import os,sys,pytest
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MR=os.environ.get("B1_ROOT",ROOT); os.environ.setdefault("PJJ",MR)      # 모델 루트: 재빌드 폴더(build/) 또는 저장소 그대로
V1=os.environ.get("B1_V1",os.path.join(MR,"b1","models","v1"))
for p in (os.path.join(ROOT,"config"),os.path.join(ROOT,"tests","support"),os.path.join(V1,"scripts"),os.path.join(MR,"scenarios")): sys.path.insert(0,p)
import params as PRM

def pytest_configure(config): config.addinivalue_line("markers","slow: 집기 시나리오 롤아웃(수십 초 ~ 수 분)")

@pytest.fixture(scope="session")
def par(): return PRM.load()
@pytest.fixture(scope="session")
def scene(): return os.path.join(V1,"mjcf","scene.xml")
@pytest.fixture(scope="session")
def arm():
    import numpy as np, w10_common as C
    A=C.Arm(); Q=A.sample(3000,0); T=[A.terms(q) for q in Q]
    return dict(C=C,A=A,Q=Q,TG=np.array([t[0] for t in T]),SZ=np.array([t[1] for t in T]),RHO=np.array([t[4] for t in T]))
@pytest.fixture(scope="session")
def pick():
    import expert as E
    E.set_limits(None,None); m=E.build(); yield dict(E=E,m=m); E.set_limits(None,None)

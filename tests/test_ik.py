# IK 회귀 테스트 (week10, 박종진) — wb_ik.py(인터페이스 v2 + reach_limit) · 6 · 7주차 숫자 고정
#   통과 기준과 허용 오차 이유
#   ① 13목표(wb_ik_compare): 전신 성공 = 정확히 10개, 실패 3개 = 높이(T10 · A1 · A2) — 결정적 계산(같은 시작값 · 같은 반복식)이라 개수 · 이름 모두 정확히
#   ② 무작위 300(rng 7) 위치 IK 전신 수렴률 ≥ 99 %(기준 100 %): 1 % = 3목표 — 수치 미세 차이(mujoco · numpy 버전)로 경계 목표 1 ~ 2개가 바뀔 수 있는 폭
#   ③ 무작위 150 위치 + 아래로 수렴률 78 ± 2 %p(기준 78.0 %): 실패는 수직 하강 대역 위(z > 1.04 m) = 기구 한계 → 2 %p(3목표)보다 크게 바뀌면 모델(관절 원점 · 범위)이 바뀐 것
#   ④ reach_of = 함태훈 w10_common rho(같은 정의) 1e-9 m 이내 — 정의가 하나여야 r_max 를 그대로 씀
#   ⑤ reach_limit 이 안 걸리면(r_max ≥ 해의 r) 결과가 제한 없음과 비트 단위 같음 / 걸리면 해의 r ≤ r_max + 1 mm, 베이스가 더 움직임
#   ⑥ load_reach_limit: params 기본(꺼짐) → None, auto · a2_doc · 3 kg → None(최대 도달), auto · cad_ri85 · 5 kg → 0.63 m(함태훈 표)
import copy,json,os,numpy as np,pytest,mujoco

@pytest.fixture(scope="module")
def b1():
    from ik import load_b1
    return load_b1()

def test_1_thirteen_targets(b1,record_property):
    from wb_ik import solve_wb_ik
    m,d=b1; EDGE=0.288; DOWN=[0,0,-1]; FWD=[1,0,0]
    T=[("T9",[0.80,0.25,1.20],None,None),("T10",[0.20,0.20,0.42],None,None),("X0",[0.55,0.28,1.20],FWD,None),("X2",[0.30,0.25,0.85],DOWN,None),
       ("A1",[0.25,0.28,0.55],None,None),("A2",[0.60,0.28,0.05],None,None),("A3",[0.60,0.28,0.75],None,None),("A4",[EDGE+0.15,0.28,0.75],None,EDGE),
       ("A5",[EDGE+0.30,0.28,0.75],None,EDGE),("A6",[EDGE+0.30,0.28,1.10],None,EDGE),("A7",[0.35,-0.30,1.00],None,None),("A8",[0.45,0.28,0.80],DOWN,None),
       ("A9",[EDGE+0.10,0.28,0.80],DOWN,EDGE)]
    st={n:solve_wb_ik(m,d,"left",p,ax,table_x=tx,restarts=8).status for n,p,ax,tx in T}
    ok=[n for n,s in st.items() if s=="converged"]; record_property("wb_ok",f"{len(ok)}/13")
    assert len(ok)==10
    assert {n:s for n,s in st.items() if s!="converged"}=={"T10":"unreachable_height","A1":"unreachable_height","A2":"unreachable_height"}

def _rates(m,d):
    from wb_ik import solve_wb_ik,Z_BAND
    rng=np.random.default_rng(7); N=300
    P=np.column_stack([rng.uniform(0.15,1.10,N),rng.uniform(-0.60,0.90,N),rng.uniform(Z_BAND[0]+0.02,Z_BAND[1]-0.02,N)])
    # wb_ik_compare.py 와 같은 난수 순서(P 300 → Pd 150) → 같은 목표 집합
    Pd=np.column_stack([rng.uniform(0.25,1.0,150),rng.uniform(-0.3,0.8,150),rng.uniform(0.62,1.10,150)])
    a=np.mean([solve_wb_ik(m,d,"left",p,None,restarts=4).success for p in P])*100
    b=np.mean([solve_wb_ik(m,d,"left",p,[0,0,-1],restarts=4).success for p in Pd])*100
    return a,b

@pytest.mark.slow
def test_2_3_random_convergence(b1,record_property):
    a,b=_rates(*b1); record_property("pos_rate",f"{a:.1f} %"); record_property("down_rate",f"{b:.1f} %")
    assert a>=99.0
    assert abs(b-78.0)<=2.0

def test_4_reach_definition_matches_taehoon(arm,record_property):
    from wb_ik import reach_of
    A=arm["A"]; m=A.m; d=mujoco.MjData(m); err=0.0
    for q,rho in zip(arm["Q"][:200],arm["RHO"][:200]):
        d.qpos[:]=m.qpos0; d.qpos[A.qa]=q; mujoco.mj_kinematics(m,d); err=max(err,abs(reach_of(m,d,"left")-rho))
    record_property("max_diff_m",f"{err:.1e}"); assert err<1e-9
    assert float(arm["RHO"].max())==pytest.approx(0.729,abs=1e-3)      # 함태훈 최대 도달 0.729 m

def test_5_reach_limit_noop_and_binding(b1,record_property):
    from wb_ik import solve_wb_ik
    m,d=b1; p=[0.60,0.28,0.75]                                         # A3: 정면 테이블 높이, 베이스가 다가가야 닿음
    r0=solve_wb_ik(m,d,"left",p,None,restarts=8)
    r1=solve_wb_ik(m,d,"left",p,None,restarts=8,reach_limit=r0.reach+0.05)      # 안 걸림 → 사전 배치가 frac 0.8 이상이면 그대로여야 함
    assert r0.success and r1.success
    rl=0.30; r2=solve_wb_ik(m,d,"left",p,None,restarts=8,reach_limit=rl)
    record_property("A3_reach_free_m",round(r0.reach,3)); record_property("A3_reach_lim_m",round(r2.reach,3))
    record_property("A3_base_move_free_lim_m",f"{r0.base_move:.3f} → {r2.base_move:.3f}")
    assert r2.success and r2.reach<=rl+1e-3 and r2.base_move>r0.base_move
    assert r1.pos_err<=1e-3 and r1.reach<=r0.reach+0.05+1e-3

def test_5b_unbound_bit_identical(b1):
    from wb_ik import solve_wb_ik
    m,d=b1
    for p in ([0.80,0.25,1.20],[0.35,-0.30,1.00]):
        a=solve_wb_ik(m,d,"left",p,None,restarts=8); b=solve_wb_ik(m,d,"left",p,None,restarts=8,reach_limit=None)
        assert np.array_equal(a.q,b.q) and np.array_equal(a.base,b.base) and a.status==b.status

def test_6_load_reach_limit(par):
    from wb_ik import load_reach_limit
    assert load_reach_limit(par) is None                                # 기본 꺼짐(결정 전)
    p=copy.deepcopy(par); p["limits"]["reach_limit"]["enable"]=True
    assert load_reach_limit(p) is None                                  # a2_doc(TQ) 3 kg → 최대 도달까지 OK
    assert load_reach_limit(p,case="cad_ri85",payload=3.0) is None      # RI85 3 kg 도 OK
    assert load_reach_limit(p,case="cad_ri85",payload=5.0)==pytest.approx(0.63)
    assert load_reach_limit(p,case="cad_ri85",payload=4.8)==pytest.approx(0.63)   # 표 사이 → 무거운 쪽(보수적)
    p["limits"]["reach_limit"]["r_max_m"]=0.5; assert load_reach_limit(p)==0.5

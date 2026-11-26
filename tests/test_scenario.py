# 집기 시나리오 회귀 테스트 (week10, 박종진) — scenarios/expert.py(전문가 정책) · 제한 플래그
#   통과 기준과 허용 오차 이유
#   ① 무작위 20회(seed 0 ~ 19, seed 질량) 성공 ≥ 19/20, 실패는 seed 1 ik_fail 하나만(8주차 run_pick20 기준) — 결정적 시뮬(같은 seed · dt 0.5 ms)이라
#      개수가 바뀌면 원인이 있음. 허용 오차 0(19 미만이면 실패), 다만 20/20 이 되는 개선은 통과
#   ② 성공 에피소드 목표 오차 평균 ≤ 8 mm · 최대 ≤ 20 mm (8주차 6.0 · 13.6 mm): 목표 영역 반지름 40 mm 의 1/5 · 1/2 — 놓기 정밀도가 이 이상 나빠지면 제어 · 접촉 변경
#   ③ 제한 플래그가 안 걸리는 값(params auto: m_max 11.04 kg · r_max 0.729 m)이면 결과가 제한 없음과 같음(사이클 시간 · 목표 오차 동일) — 플래그 구현이 동작을 바꾸지 않는지
#   ④ 도달 제한이 걸리는 값(r_max 0.37 m)이면 운반 중 실측 r ≤ r_max + 15 mm — 15 mm = 계획은 r ≤ r_max 로 투영하지만 실제 팔 · 베이스 추종 오차(8주차 최대 13.6 mm) 만큼 넘을 수 있음
#   ⑤ 짐 제한(m_max 2 kg)이면 물체 질량 = min(3, 2) = 2 kg
import numpy as np,pytest

def _run(pick,seed,mass=None,reach=None,payload=None,track=False):
    E=pick["E"]; m=pick["m"]; E.set_limits(reach,payload); R=[]
    from wb_ik import reach_of
    cb=(lambda d,a,ex: R.append(reach_of(m,d,"left")) if ex.stage in ("LIFT","MOVE","PLACE") else None) if track else None
    try: res,_=E.run_episode(m,seed,mass=mass,on_step=cb)
    finally: E.set_limits(None,None)
    return res,(max(R) if R else None)

@pytest.mark.slow
def test_1_2_pick20(pick,record_property):
    rows=[_run(pick,sd)[0] for sd in range(20)]; ok=[r for r in rows if r["success"]]
    fails={r["seed"]:r["fail"] for r in rows if not r["success"]}
    e=[r["goal_err_mm"] for r in ok]
    record_property("success",f"{len(ok)}/20"); record_property("fails",str(fails)); record_property("goal_err_mm",f"{np.mean(e):.1f} / {np.max(e):.1f}")
    record_property("t_cycle_s",f"{np.mean([r['t_total'] for r in ok]):.1f}")
    assert len(ok)>=19 and set(fails.values())<= {"ik_fail"} and set(fails)<= {1}
    assert np.mean(e)<=8.0 and np.max(e)<=20.0

@pytest.mark.slow
@pytest.mark.parametrize("seed",[0,3])
def test_3_flags_noop(pick,seed,record_property):
    a,_=_run(pick,seed,mass=3.0); b,_=_run(pick,seed,mass=3.0,payload=11.04); c,_=_run(pick,seed,mass=3.0,reach=0.729)
    record_property("t_total",f"{a['t_total']} / {b['t_total']} / {c['t_total']}")
    for x in (b,c): assert (x["success"],x["t_total"],x["goal_err_mm"])==(a["success"],a["t_total"],a["goal_err_mm"])

@pytest.mark.slow
def test_4_reach_binding(pick,record_property):
    res,rc=_run(pick,13,mass=3.0,reach=0.37,track=True); free,rf=_run(pick,13,mass=3.0,track=True)   # seed 13: 제한 없으면 운반 중 r 0.378 m
    record_property("seed13_reach_carry_free_lim",f"{rf:.3f} → {rc:.3f}"); record_property("success_lim",res["success"])
    assert rf>0.37                       # 제한 없으면 0.37 을 넘는 자세로 운반
    assert res["success"] and rc<=0.37+0.015

def test_5_payload_flag(pick):
    E=pick["E"]; m=pick["m"]; E.set_limits(None,2.0)
    try: res,_=E.run_episode(m,0,mass=3.0,max_t=0.05)     # 질량만 확인(0.05 s 후 중단)
    finally: E.set_limits(None,None)
    assert res["mass"]==pytest.approx(2.0)

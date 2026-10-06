"""week03 수요일: position 액추에이터 계단 응답 — kp · kv · armature 비교

실행 (저장소 루트에서):
    python members/park-jongjin/week03/scripts/wed_step_response.py
실험 조건:
    - 모델: mjcf/scene_position.xml (2링크 팔, 중력 있음, implicitfast, dt 0.002 s)
    - t < 0.5 s : ctrl = (0, 0) 유지 (중력으로 처진 평형까지 정착)
    - t = 0.5 s : 어깨 목표를 0 → −0.5 rad (위로 들어 올림) 계단 입력, 팔꿈치 목표 0 유지
    - 세 실험: (A) kp 변화, (B) kv 변화, (C) armature 변화. 나머지는 기본값(kp 50, kv 1, armature 0)
지표 (어깨, 계단 이후 구간):
    rise 10–90 %, 오버슈트 %, 2 % 정착시간, 정상상태 오차(최종 q − 목표, + = 중력 방향으로 처짐), 최대 액추에이터 힘
이론 비교 (어깨 1자유도 근사):
    M = 질량행렬 M[0,0](q=목표 자세) + armature,  ωn = √(kp/M),  ζ = (kv + damping)/(2√(kp·M))
    정상상태 오차 ≈ τ_gravity / kp
출력: results/wed_step_metrics.csv, results/wed_step.txt, img/20261006_step_{kp,kv,armature}.png, img/20261006_step_all.png
"""
import os, io, csv
import numpy as np
import mujoco
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); W3 = os.path.dirname(HERE)
XML = os.path.join(W3, "mjcf", "scene_position.xml")
RES = os.path.join(W3, "results"); IMG = os.path.join(W3, "img")
T_STEP, T_END, Q0, Q1 = 0.5, 3.0, 0.0, -0.5
out = io.StringIO()
def p(*a): print(*a); print(*a, file=out)

def run(kp=50.0, kv=1.0, arm=0.0):
    m = mujoco.MjModel.from_xml_path(XML)
    for a in range(m.nu):
        m.actuator_gainprm[a, 0] = kp; m.actuator_biasprm[a, 1] = -kp; m.actuator_biasprm[a, 2] = -kv
    m.dof_armature[:] = arm
    d = mujoco.MjData(m)
    n = int(round(T_END / m.opt.timestep)); log = np.zeros((n, 4))
    for i in range(n):
        d.ctrl[:] = [Q0 if d.time < T_STEP else Q1, 0.0]
        mujoco.mj_step(m, d)
        log[i] = [d.time, d.qpos[0], d.qpos[1], d.actuator_force[0]]
    # 이론값: 목표 자세에서의 질량행렬 · 중력토크
    d2 = mujoco.MjData(m); d2.qpos[:] = [Q1, 0]; mujoco.mj_forward(m, d2)
    M = np.zeros((m.nv, m.nv))
    try:    mujoco.mj_fullM(m, d2, M)       # MuJoCo 3.15+: (m, d, dst)
    except TypeError: mujoco.mj_fullM(m, M, d2.qM)  # 이전 버전: (m, dst, qM)
    Meff = M[0, 0]; tg = d2.qfrc_bias[0]          # armature 포함 (mj_fullM 은 armature 포함)
    damp = m.dof_damping[0]
    z = (kv + damp) / (2 * np.sqrt(kp * Meff))
    os_th = 100 * np.exp(-np.pi * z / np.sqrt(1 - z * z)) if z < 1 else 0.0   # 2차계 오버슈트 공식
    return log, dict(os_th=os_th, M=Meff, wn=np.sqrt(kp / Meff), zeta=(kv + damp) / (2 * np.sqrt(kp * Meff)), ess_th=-tg / kp)

def metrics(log):
    t, q, f = log[:, 0], log[:, 1], log[:, 3]
    k = t >= T_STEP; t, q, f = t[k] - T_STEP, q[k], f[k]
    q_start, q_final = q[0], q[-200:].mean()
    amp = q_final - q_start
    frac = (q - q_start) / amp
    t10 = t[np.argmax(frac >= 0.1)]; t90 = t[np.argmax(frac >= 0.9)]
    over = max(0.0, (np.max(frac) - 1.0) * 100)
    band = np.abs(q - q_final) > 0.02 * abs(amp)
    last = np.where(band)[0][-1] if band.any() else -1
    ts = float('nan') if last >= len(t) - 1 else (t[last + 1] if last >= 0 else 0.0)   # nan = 구간 안에 정착 못 함
    return dict(q_start=q_start, q_final=q_final, rise=t90 - t10, over=over, settle=ts,
                ess=q_final - Q1, fmax=np.max(np.abs(f)))

EXPS = {
    "kp": ("(A) kp 변화 (kv=1, armature=0)", [dict(kp=20), dict(kp=50), dict(kp=100)], lambda c: f"kp={c['kp']:g}"),
    "kv": ("(B) kv 변화 (kp=50, armature=0)", [dict(kv=0.0), dict(kv=1.0), dict(kv=5.0)], lambda c: f"kv={c['kv']:g}"),
    "armature": ("(C) armature 변화 (kp=50, kv=1)", [dict(arm=0.0), dict(arm=0.05), dict(arm=0.15)], lambda c: f"armature={c['arm']:g}"),
}
EN = {"kp": "(A) kp sweep (kv=1, armature=0)", "kv": "(B) kv sweep (kp=50, armature=0)", "armature": "(C) armature sweep (kp=50, kv=1)"}
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c"]
rows = []
fig_all, axs = plt.subplots(1, 3, figsize=(15, 3.8), sharey=True)
p(f"MuJoCo {mujoco.__version__} | 계단: 어깨 {Q0} → {Q1} rad at t={T_STEP}s | forcerange ±40 Nm")
for j, (key, (title, cases, lab)) in enumerate(EXPS.items()):
    p("\n" + title)
    p(f"{'case':16s} {'M_eff':>7s} {'ωn':>6s} {'ζ':>6s} | {'rise[s]':>7s} {'OS[%]':>6s} {'OS_th':>6s} {'ts2%[s]':>7s} {'ess[rad]':>9s} {'ess_th':>8s} {'|f|max':>7s}")
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for c, col in zip(cases, COLORS):
        log, th = run(**c); mt = metrics(log); name = lab(c)
        p(f"{name:16s} {th['M']:7.4f} {th['wn']:6.2f} {th['zeta']:6.3f} | {mt['rise']:7.3f} {mt['over']:6.1f} {th['os_th']:6.1f} {mt['settle']:7.3f} {mt['ess']:9.4f} {th['ess_th']:8.4f} {mt['fmax']:7.2f}")
        rows.append(dict(exp=key, case=name, M_eff=float(th['M']), wn=th['wn'], zeta=th['zeta'], rise=mt['rise'], overshoot_pct=mt['over'], overshoot_theory=float(th['os_th']),
                         settle_2pct=mt['settle'], ess=mt['ess'], ess_theory=th['ess_th'], fmax=mt['fmax']))
        for a in (ax, axs[j]):
            a.plot(log[:, 0], log[:, 1], color=col, lw=1.8, label=f"{name} (zeta={th['zeta']:.2f})")
    for a in (ax, axs[j]):
        a.step([0, T_STEP, T_END], [Q0, Q1, Q1], where="post", color="k", ls=":", lw=1, label="target")
        a.set_title(EN[key], fontsize=10); a.set_xlabel("time [s]"); a.grid(alpha=.3); a.legend(fontsize=7.5)
    ax.set_ylabel("shoulder q [rad]"); fig.tight_layout()
    fig.savefig(os.path.join(IMG, f"20261006_step_{key}.png"), dpi=150); plt.close(fig)
axs[0].set_ylabel("shoulder q [rad]"); fig_all.tight_layout(); fig_all.savefig(os.path.join(IMG, "20261006_step_all.png"), dpi=150)
with open(os.path.join(RES, "wed_step_metrics.csv"), "w", newline="", encoding="utf-8") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader()
    for r in rows: w.writerow({k: (f"{v:.5f}" if isinstance(v, float) else v) for k, v in r.items()})
open(os.path.join(RES, "wed_step.txt"), "w", encoding="utf-8").write(out.getvalue())

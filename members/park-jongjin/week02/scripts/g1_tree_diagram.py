"""week02 개별 과제: G1 MJCF 바디 트리를 그림으로 (우리 로봇 기준 변경 표시)
사용법: python g1_tree_diagram.py <mujoco_menagerie 경로> [출력폴더]
  -> g1_tree.dot 생성, graphviz(dot)가 있으면 g1_tree.svg / g1_tree.png 까지 렌더
색 구분 (g1-structure-tree.md 변경 목록 기준)
  빨강  : 삭제 (다리 12-DOF → 홀로노믹 베이스 평면 3-DOF로 대체)
  주황  : 교체 (루트 freejoint → 평면 가상관절 slide x · slide y · hinge z)
  회색  : 유지 여부 확인 필요 (허리 3-DOF)
  파랑  : 유지 후 치수 · 질량 · 토크 교체 (어깨 3 + 팔꿈치 1)
  보라  : 7→6-DOF 축소 시 1개 제거 후보 (손목 3)
"""
import os, sys, shutil, subprocess
import mujoco

menagerie = sys.argv[1] if len(sys.argv) > 1 else "mujoco_menagerie"
out = sys.argv[2] if len(sys.argv) > 2 else "."
os.makedirs(out, exist_ok=True)
m = mujoco.MjModel.from_xml_path(os.path.join(menagerie, "unitree_g1", "g1.xml"))
name = lambda t, i: mujoco.mj_id2name(m, t, i)

def style(body):
    if "hip" in body or "knee" in body or "ankle" in body: return "#f4c7c3", "#c0392b"
    if body == "pelvis": return "#fde3c3", "#d35400"
    if "waist" in body: return "#e5e5e5", "#777777"
    if "wrist" in body: return "#e6dcf5", "#7d3c98"
    if "shoulder" in body or "elbow" in body: return "#d6e6f7", "#2e6db4"
    return "#ffffff", "#333333"

L = ['digraph G {', 'rankdir=TB; nodesep=0.12; ranksep=0.28;',
     'node [shape=box, style="rounded,filled", fontname="Noto Sans CJK KR", fontsize=10, margin="0.08,0.03"];',
     'edge [fontname="Noto Sans CJK KR", fontsize=8, color="#555555", arrowsize=0.5];',
     'labelloc=t; fontname="Noto Sans CJK KR"; fontsize=16;',
     f'label="Unitree G1 (g1_29dof_rev_1_0) 바디 트리 — 바디 {m.nbody - 1} · 관절 {m.njnt} · {sum(m.body_mass):.2f} kg";']
for b in range(1, m.nbody):
    bn = name(mujoco.mjtObj.mjOBJ_BODY, b)
    fill, line = style(bn)
    L.append(f'"{bn}" [label="{bn}\\n{m.body_mass[b]:.3f} kg", fillcolor="{fill}", color="{line}"];')
    p = m.body_parentid[b]
    pn = "world" if p == 0 else name(mujoco.mjtObj.mjOBJ_BODY, p)
    js = [name(mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b])]
    jt = {0: "free", 3: "hinge"}
    lab = "\\n".join(f"{j} ({jt.get(int(m.jnt_type[m.body_jntadr[b]]), '?')})" for j in js) if js else "fixed"
    lab = lab.replace("_joint", "")
    L.append(f'"{pn}" -> "{bn}" [label="{lab}"];')
L.append('"world" [shape=ellipse, fillcolor="#ffffff"];')
L.append('subgraph cluster_legend { label="우리 로봇(6-DOF 팔 × 2 + 홀로노믹 베이스) 기준"; fontsize=11; style=dashed; color="#999999";'
         'l1 [label="삭제: 다리 12-DOF", fillcolor="#f4c7c3", color="#c0392b"];'
         'l2 [label="교체: freejoint → 평면 3-DOF", fillcolor="#fde3c3", color="#d35400"];'
         'l3 [label="확인 필요: 허리 3-DOF", fillcolor="#e5e5e5", color="#777777"];'
         'l4 [label="유지·치수 교체: 어깨3+팔꿈치1", fillcolor="#d6e6f7", color="#2e6db4"];'
         'l5 [label="1개 제거 후보: 손목 3", fillcolor="#e6dcf5", color="#7d3c98"];'
         'l1 -> l2 -> l3 -> l4 -> l5 [style=invis]; }')
L.append('}')
dot_path = os.path.join(out, "g1_tree.dot")
open(dot_path, "w", encoding="utf-8").write("\n".join(L))
print("생성:", dot_path)
if shutil.which("dot"):
    for fmt in ("svg", "png"):
        args = ["dot", f"-T{fmt}", dot_path, "-o", os.path.join(out, f"g1_tree.{fmt}")]
        if fmt == "png": args.insert(2, "-Gdpi=150")
        subprocess.run(args, check=True)
        print("렌더:", os.path.join(out, f"g1_tree.{fmt}"))
else:
    print("graphviz 미설치 — https://dreampuf.github.io/GraphvizOnline 에 g1_tree.dot 내용을 붙여넣어 확인")

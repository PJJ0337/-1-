"""MuJoCo 뷰어 실행기 (Windows 한글 경로 대응)

`python -m mujoco.viewer --mjcf=...` 는 경로에 한글이 있으면 파일을 못 연다 → Python으로 읽어서 뷰어에 넘김.
사용 (저장소 루트에서):
    python members/park-jongjin/week03/scripts/view.py scene
    python members/park-jongjin/week03/scripts/view.py scene_position
    python members/park-jongjin/week03/scripts/view.py holonomic_base_two_arms
(인자는 week03/mjcf/ 안의 파일 이름, .xml 생략 가능)
"""
import os, sys
import mujoco, mujoco.viewer
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjio import load_model

name = sys.argv[1] if len(sys.argv) > 1 else "scene"
if not name.endswith(".xml"): name += ".xml"
path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "mjcf", name)
print("열기:", path)
mujoco.viewer.launch(load_model(path))

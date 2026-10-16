"""v1 모델 로더 — Windows 한글 경로 대응 (v0_mjio.py와 같은 방식: xml · STL을 메모리 assets로 전달, include 포함)
사용: python v1_mjio.py ../mjcf/scene.xml            → 로드 확인 후 뷰어 (액추에이터 슬라이더 = Control 패널)
      python v1_mjio.py ../urdf/b1_v1.urdf --noview
"""
import os,sys,glob,mujoco
def _assets(path):
    root=os.path.dirname(os.path.dirname(os.path.abspath(path)))   # v1 폴더
    a={}
    for f in glob.glob(os.path.join(root,"mjcf","*.xml")): a[os.path.basename(f)]=open(f,"rb").read()
    for f in glob.glob(os.path.join(root,"meshes","*.stl")): a[os.path.basename(f)]=open(f,"rb").read()
    return a
def load_model(path):
    return mujoco.MjModel.from_xml_string(open(path,encoding="utf-8").read(),assets=_assets(path))
if __name__=="__main__":
    m=load_model(sys.argv[1]); print("loaded",os.path.basename(sys.argv[1]),"nq",m.nq,"nu",m.nu,"nsensor",m.nsensor,"mass %.3f"%sum(m.body_mass))
    if "--noview" not in sys.argv:
        import mujoco.viewer; mujoco.viewer.launch(m)

"""v0 모델 로더 — Windows 한글 경로 대응 (week03 mjio.py 확장: STL 메시까지 assets로 전달)
사용: python v0_mjio.py <model 폴더>/mjcf/scene.xml   → 로드 확인 후 뷰어 실행
      python v0_mjio.py <model 폴더>/urdf/b1_humanoid_v0.urdf --noview
"""
import os,sys,glob,mujoco
def _assets(path):
    root=os.path.dirname(os.path.dirname(os.path.abspath(path)))  # model 폴더
    a={}
    for f in glob.glob(os.path.join(root,"mjcf","*.xml")): a[os.path.basename(f)]=open(f,"rb").read()
    for f in glob.glob(os.path.join(root,"meshes","*.stl")):
        b=open(f,"rb").read(); n=os.path.basename(f)
        a[n]=b      # 파일명만 키로 (meshdir 접두어 불필요, MuJoCo 3.15 확인)
    return a
def load_model(path):
    return mujoco.MjModel.from_xml_string(open(path,encoding="utf-8").read(),assets=_assets(path))
if __name__=="__main__":
    m=load_model(sys.argv[1]); print("loaded",os.path.basename(sys.argv[1]),"nq",m.nq,"nu",m.nu,"nmesh",m.nmesh,"mass %.2f"%sum(m.body_mass))
    if "--noview" not in sys.argv:
        import mujoco.viewer; mujoco.viewer.launch(m)

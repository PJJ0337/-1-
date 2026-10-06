"""한글(비 ASCII) 경로 우회용 MuJoCo 입출력 도우미

Windows에서 MuJoCo는 파일을 C 함수로 직접 여는데, 경로에 한글(예: '바탕 화면', '캡스톤 디자인')이 있으면
'ParseXML: Error opening file' 로 실패한다 (2026-10-06 노트북 재현 중 발견, Linux에서는 정상).
→ 파일은 Python이 UTF-8로 읽고, MuJoCo에는 문자열 + assets(include 대상 파일)로 넘긴다.
   저장도 ASCII 임시 폴더에 쓴 뒤 Python으로 옮긴다.
"""
import os, glob, shutil, tempfile
import mujoco

def _assets(folder):
    # 같은 폴더의 xml/urdf 를 include 대상으로 전부 등록 (scene.xml → two_link_arm.xml 같은 include 해결)
    out = {}
    for f in glob.glob(os.path.join(folder, "*.xml")) + glob.glob(os.path.join(folder, "*.urdf")):
        with open(f, "rb") as fh:
            out[os.path.basename(f)] = fh.read()
    return out

def read_text(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()

def load_model(path):
    """MjModel.from_xml_path 대체"""
    return mujoco.MjModel.from_xml_string(read_text(path), assets=_assets(os.path.dirname(os.path.abspath(path))))

def load_spec(path):
    """MjSpec.from_file 대체"""
    return mujoco.MjSpec.from_string(read_text(path), assets=_assets(os.path.dirname(os.path.abspath(path))))

def save_last_xml(path, m):
    """mj_saveLastXML 대체: ASCII 임시 경로에 저장 후 이동"""
    fd, tmp = tempfile.mkstemp(suffix=".xml"); os.close(fd)
    mujoco.mj_saveLastXML(tmp, m)
    shutil.move(tmp, path)

"""week03 월요일 실습: usd-core로 큐브 1개짜리 .usda 생성 후 텍스트로 구조 확인
사용법: python make_cube_usda.py [출력폴더]
개념: Stage(파일 전체 = 루트 레이어) > Prim(경로로 식별되는 노드) > Attribute(값) / Metadata
"""
import os, sys
from pxr import Usd, UsdGeom, Gf, Sdf

out = sys.argv[1] if len(sys.argv) > 1 else "usd"
os.makedirs(out, exist_ok=True)
path = os.path.join(out, "cube.usda")

stage = Usd.Stage.CreateNew(path)                 # Stage + 루트 Layer 생성
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)   # Isaac Sim 규약: Z-up
UsdGeom.SetStageMetersPerUnit(stage, 1.0)         # 1 unit = 1 m

world = UsdGeom.Xform.Define(stage, "/World")     # Prim: 타입 Xform
stage.SetDefaultPrim(world.GetPrim())             # reference 시 기본으로 가져올 prim

cube = UsdGeom.Cube.Define(stage, "/World/Cube")  # Prim: 타입 Cube
cube.GetSizeAttr().Set(0.2)                       # Attribute: size = 0.2 m (한 변)
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.1))    # xformOp:translate → 바닥 위에 놓기
cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.5, 0.9)])
cube.GetPrim().SetCustomDataByKey("note", "week03 cube")   # 사용자 메타데이터

stage.GetRootLayer().Save()
print(open(path, encoding="utf-8").read())

# 다시 열어서 순회
s = Usd.Stage.Open(path)
print("upAxis:", UsdGeom.GetStageUpAxis(s), "metersPerUnit:", UsdGeom.GetStageMetersPerUnit(s))
for prim in s.Traverse():
    print(prim.GetPath(), prim.GetTypeName(), [a.GetName() for a in prim.GetAuthoredAttributes()])

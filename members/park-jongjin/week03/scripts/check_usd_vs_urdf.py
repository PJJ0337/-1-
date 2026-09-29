"""week03 검증: 합성된 scene.usda의 2링크 팔이 week02 URDF와 같은 정보를 담고 있는지 자동 점검
(Isaac Sim 없이 usd-core만으로 확인 가능한 항목)
사용법: python check_usd_vs_urdf.py [scene.usda] [two_link_arm.urdf]
"""
import os, sys, math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import yourdfpy

here = os.path.dirname(os.path.abspath(__file__))
scene_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "usd", "scene.usda")
urdf_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "..", "..", "week02", "models", "two_link_arm.urdf")

stage = Usd.Stage.Open(scene_path)
urdf = yourdfpy.URDF.load(urdf_path)
ok = True
def check(cond, msg):
    global ok
    ok &= bool(cond)
    print(("  [OK]   " if cond else "  [FAIL] ") + msg)

cache = UsdGeom.XformCache()
def world_point(prim_path, local):
    if not prim_path:
        return Gf.Vec3d(*local)
    M = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))
    return M.Transform(Gf.Vec3d(*local))

print("1) 합성(composition) · 스키마")
arm = stage.GetPrimAtPath("/World/two_link_arm")
check(arm.IsValid() and arm.HasAuthoredReferences(), "/World/two_link_arm 가 reference로 합성됨")
bodies = [p for p in Usd.PrimRange(arm) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
roots = [p for p in Usd.PrimRange(arm) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
check(len(bodies) == 3, f"RigidBody 3개: {[b.GetName() for b in bodies]}")
check(len(roots) == 1, f"ArticulationRoot 정확히 1개 (중첩 불가): {[str(r.GetPath()) for r in roots]}")
check(stage.GetPrimAtPath("/World/physicsScene").IsA(UsdPhysics.Scene), "PhysicsScene 존재")
check(UsdGeom.GetStageUpAxis(stage) == "Z" and UsdGeom.GetStageMetersPerUnit(stage) == 1.0, "Z-up, metersPerUnit=1")

print("2) 질량 (URDF 대비)")
usd_mass = {b.GetName(): UsdPhysics.MassAPI(b).GetMassAttr().Get() for b in bodies}
for name, mval in usd_mass.items():
    um = urdf.link_map[name].inertial.mass
    check(abs(mval - um) < 1e-6, f"{name}: USD {mval} kg / URDF {um} kg")
check(abs(sum(usd_mass.values()) - 3.6) < 1e-6, f"총질량 {sum(usd_mass.values()):.3f} kg")

print("3) 관절 (URDF 대비) · 관절 프레임 일치")
for j in [p for p in Usd.PrimRange(arm) if p.IsA(UsdPhysics.RevoluteJoint)]:
    rj = UsdPhysics.RevoluteJoint(j)
    b0 = rj.GetBody0Rel().GetTargets(); b1 = rj.GetBody1Rel().GetTargets()
    p0 = world_point(str(b0[0]) if b0 else "", rj.GetLocalPos0Attr().Get())
    p1 = world_point(str(b1[0]), rj.GetLocalPos1Attr().Get())
    uj = urdf.joint_map[j.GetName()]
    lo, hi = rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()
    axis = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[rj.GetAxisAttr().Get()]
    check((p0 - p1).GetLength() < 1e-6, f"{j.GetName()}: body0·localPos0 = body1·localPos1 = {np.round(list(p0), 4)} (world)")
    check(np.allclose(rj.GetLocalPos0Attr().Get(), uj.origin[:3, 3]), f"{j.GetName()}: localPos0 = URDF origin xyz {uj.origin[:3, 3]}")
    check(np.allclose(axis, uj.axis), f"{j.GetName()}: axis {rj.GetAxisAttr().Get()} = URDF {uj.axis}")
    check(abs(math.radians(lo) - uj.limit.lower) < 1e-4 and abs(math.radians(hi) - uj.limit.upper) < 1e-4,
          f"{j.GetName()}: limit {lo:.2f}~{hi:.2f} deg = URDF {uj.limit.lower}~{uj.limit.upper} rad")
    mf = UsdPhysics.DriveAPI(j, "angular").GetMaxForceAttr().Get()
    check(abs(mf - uj.limit.effort) < 1e-6, f"{j.GetName()}: drive maxForce {mf} = URDF effort {uj.limit.effort}")

print("4) 말단(tool0) 위치")
tool = world_point("/World/two_link_arm/base_link/link1/link2/tool0", (0, 0, 0))
check(np.allclose(list(tool), [0.55, 0, 0.10]), f"tool0 = {np.round(list(tool), 4)} (URDF FK: [0.55, 0, 0.10])")

print("\n결과:", "모두 통과" if ok else "실패 항목 있음")
sys.exit(0 if ok else 1)

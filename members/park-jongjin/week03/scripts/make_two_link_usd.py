"""week03 화·수 실습: 2링크 팔을 USD로 (week02 two_link_arm.urdf와 같은 치수·질량)
  화) two_link_arm_geom.usda    : Xform 계층 + 형상만 (물리 없음)
  수) two_link_arm_physics.usda : geom 레이어를 sublayer로 깔고 물리 스키마를 over로 덧씀
                                  RigidBody · Mass · Collision · RevoluteJoint 2 · DriveAPI · ArticulationRoot
      scene.usda                : PhysicsScene + 바닥 + 팔을 reference로 불러옴
사용법: python make_two_link_usd.py [출력폴더]
주의: USD Physics 각도 단위는 degree (URDF는 rad)
"""
import os, sys, math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

out = sys.argv[1] if len(sys.argv) > 1 else "usd"
os.makedirs(out, exist_ok=True)
R = "/two_link_arm"

# URDF와 동일한 파라미터 (week02/models/two_link_arm.urdf)
BASE = dict(r=0.05, L=0.10, m=2.0)
L1 = dict(r=0.03, L=0.30, m=1.0)
L2 = dict(r=0.025, L=0.25, m=0.6)
SHOULDER = dict(origin=(0, 0, 0.10), lower=-1.5708, upper=1.5708, effort=40, vel=3.0, damping=0.05)
ELBOW = dict(origin=(0.30, 0, 0), lower=-2.3562, upper=2.3562, effort=20, vel=3.0, damping=0.05)

def new_stage(name):
    s = Usd.Stage.CreateNew(os.path.join(out, name))
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(s, 1.0)
    return s

# ------------------------------------------------------------------ 화: 형상 + Xform 계층
g = new_stage("two_link_arm_geom.usda")
root = UsdGeom.Xform.Define(g, R); g.SetDefaultPrim(root.GetPrim())

def link(path, translate, cyl, cyl_axis, cyl_center, color):
    x = UsdGeom.Xform.Define(g, path)
    if translate is not None:
        x.AddTranslateOp().Set(Gf.Vec3d(*translate))       # = URDF joint origin (부모 기준)
    c = UsdGeom.Cylinder.Define(g, path + "/geom")
    c.GetRadiusAttr().Set(cyl["r"]); c.GetHeightAttr().Set(cyl["L"]); c.GetAxisAttr().Set(cyl_axis)
    c.AddTranslateOp().Set(Gf.Vec3d(*cyl_center))          # = URDF visual origin
    c.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return x

link(f"{R}/base_link", None, BASE, "Z", (0, 0, BASE["L"] / 2), (0.3, 0.3, 0.3))
link(f"{R}/base_link/link1", SHOULDER["origin"], L1, "X", (L1["L"] / 2, 0, 0), (0.8, 0.4, 0.1))
link(f"{R}/base_link/link1/link2", ELBOW["origin"], L2, "X", (L2["L"] / 2, 0, 0), (0.2, 0.6, 0.8))
t = UsdGeom.Xform.Define(g, f"{R}/base_link/link1/link2/tool0"); t.AddTranslateOp().Set(Gf.Vec3d(L2["L"], 0, 0))
g.GetRootLayer().Save()

# ------------------------------------------------------------------ 수: 물리 레이어 (sublayer + over)
p = new_stage("two_link_arm_physics.usda")
p.GetRootLayer().subLayerPaths.append("./two_link_arm_geom.usda")   # 아래 레이어의 prim을 그대로 보고
p.SetDefaultPrim(p.GetPrimAtPath(R))                                # 위 레이어에서 속성만 덧씀(over)

def cyl_inertia(c, along):  # 속이 찬 원통, COM 기준 주관성모멘트
    ia = c["m"] * c["r"] ** 2 / 2
    it = c["m"] * (3 * c["r"] ** 2 + c["L"] ** 2) / 12
    return Gf.Vec3f(ia, it, it) if along == "X" else Gf.Vec3f(it, it, ia)

for path, c, along, com in [(f"{R}/base_link", BASE, "Z", (0, 0, BASE["L"] / 2)),
                            (f"{R}/base_link/link1", L1, "X", (L1["L"] / 2, 0, 0)),
                            (f"{R}/base_link/link1/link2", L2, "X", (L2["L"] / 2, 0, 0))]:
    prim = p.OverridePrim(path)
    UsdPhysics.RigidBodyAPI.Apply(prim)          # 중첩된 RigidBody는 서로 독립적으로 움직임 (연결은 Joint가 담당)
    mass = UsdPhysics.MassAPI.Apply(prim)        # 명시 mass > density 추정 (USD Physics 우선순위)
    mass.CreateMassAttr(c["m"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    mass.CreateDiagonalInertiaAttr(cyl_inertia(c, along))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    UsdPhysics.CollisionAPI.Apply(p.OverridePrim(path + "/geom"))

J = f"{R}/joints"
UsdGeom.Scope.Define(p, J)

# 고정 베이스: world(body0 비움) ↔ base_link FixedJoint, 그 joint에 ArticulationRoot
fj = UsdPhysics.FixedJoint.Define(p, f"{J}/root_joint")
fj.CreateBody1Rel().SetTargets([f"{R}/base_link"])
UsdPhysics.ArticulationRootAPI.Apply(fj.GetPrim())

def revolute(name, parent, child, spec):
    j = UsdPhysics.RevoluteJoint.Define(p, f"{J}/{name}")
    j.CreateBody0Rel().SetTargets([parent]); j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(Gf.Vec3f(*spec["origin"]))   # 부모 좌표계에서 관절 위치 (= URDF joint origin)
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))           # 자식 좌표계 원점 = 관절 위치
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateAxisAttr("Y")                              # = URDF axis 0 1 0
    j.CreateLowerLimitAttr(math.degrees(spec["lower"]))   # rad → deg !
    j.CreateUpperLimitAttr(math.degrees(spec["upper"]))
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(10.0)                        # 위치 드라이브 게인 (단위: 토크/deg) — 예시값
    d.CreateDampingAttr(1.0)
    d.CreateMaxForceAttr(spec["effort"])               # = URDF limit effort
    d.CreateTargetPositionAttr(0.0)
    return j

revolute("shoulder", f"{R}/base_link", f"{R}/base_link/link1", SHOULDER)
revolute("elbow", f"{R}/base_link/link1", f"{R}/base_link/link1/link2", ELBOW)
p.GetRootLayer().Save()

# ------------------------------------------------------------------ 장면: reference로 조립
s = new_stage("scene.usda")
w = UsdGeom.Xform.Define(s, "/World"); s.SetDefaultPrim(w.GetPrim())
scene = UsdPhysics.Scene.Define(s, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); scene.CreateGravityMagnitudeAttr(9.81)
ground = UsdGeom.Cube.Define(s, "/World/ground"); ground.GetSizeAttr().Set(1.0)
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.005)); ground.AddScaleOp().Set(Gf.Vec3f(2, 2, 0.01))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
arm = s.DefinePrim("/World/two_link_arm")
arm.GetReferences().AddReference("./two_link_arm_physics.usda")   # defaultPrim(/two_link_arm)을 이 경로로 가져옴
s.GetRootLayer().Save()
print("생성:", *[os.path.join(out, f) for f in ("two_link_arm_geom.usda", "two_link_arm_physics.usda", "scene.usda")], sep="\n  ")

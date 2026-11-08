"""A 1:4 attendant (transit) wheelchair as one free articulation: a chassis (frame, seat, backrest, push handles, tipping
lever, footrest) on two large rear wheels and two small front casters, each a free revolute spin joint about the
chair's y axis. The casters don't swivel. Chair frame: origin on the table under the rear axle's middle, +x toward
the front. Dimensions in metres. Pure geometry here; build() needs Kit's pxr."""

REAR = dict(r=0.035, y=0.078, w=0.010, mass=0.04)
CASTER = dict(r=0.012, x=0.120, y=0.055, w=0.010, mass=0.01)
CHASSIS_MASS = 0.30
COM = (-0.005, 0.0, 0.070)  # with the dummy on the seat, the whole chair's centre of mass sits 12 mm ahead of the rear axle
MU = dict(wheel=1.0, body=0.5)
ITERS, VEL_ITERS = 16, 4
COLORS = {"frame": (0.55, 0.57, 0.60), "seat": (0.12, 0.30, 0.65), "tyre": (0.08, 0.08, 0.09), "lever": (0.85, 0.40, 0.05),
          "handle": (0.10, 0.10, 0.10)}

# boxes in the chassis frame: name -> (lo, hi, look)
BOXES = {
    "seat": ((0.000, -0.060, 0.065), (0.100, 0.060, 0.075), "seat"),
    "back": ((-0.012, -0.060, 0.075), (0.000, 0.060, 0.160), "seat"),
    "side_l": ((-0.012, 0.060, 0.040), (0.105, 0.066, 0.075), "frame"),
    "side_r": ((-0.012, -0.066, 0.040), (0.105, -0.060, 0.075), "frame"),
    "axle": ((-0.006, -0.072, 0.029), (0.006, 0.072, 0.041), "frame"),
    "fork_l": ((0.114, 0.064, 0.012), (0.126, 0.070, 0.045), "frame"),
    "fork_r": ((0.114, -0.070, 0.012), (0.126, -0.064, 0.045), "frame"),
    "front_l": ((0.100, 0.060, 0.040), (0.126, 0.070, 0.050), "frame"),
    "front_r": ((0.100, -0.070, 0.040), (0.126, -0.060, 0.050), "frame"),
    "footrest": ((0.115, -0.045, 0.026), (0.150, 0.045, 0.032), "frame"),
    "foot_post": ((0.100, -0.010, 0.030), (0.118, 0.010, 0.066), "frame"),
    "handle_l": ((-0.050, 0.050, 0.150), (-0.012, 0.062, 0.160), "handle"),
    "handle_r": ((-0.050, -0.062, 0.150), (-0.012, -0.050, 0.160), "handle"),
    "push_bar": ((-0.050, -0.050, 0.150), (-0.040, 0.050, 0.160), "handle"),  # joins the handles' ends
    # a tipping lever on each side, back from the frame, joined at their ends by a bar that is pressed down to tip the chair back
    "lever_arm_l": ((-0.085, 0.058, 0.034), (-0.006, 0.066, 0.040), "frame"),
    "lever_arm_r": ((-0.085, -0.066, 0.034), (-0.006, -0.058, 0.040), "frame"),
    "tip_bar": ((-0.110, -0.070, 0.025), (-0.080, 0.070, 0.040), "lever"),  # joins the two levers' ends
}
LEVER_TOP = (-0.095, 0.0, 0.040)  # the tipping bar's top centre
BAR_BACK, BAR_Z = -0.050, 0.155  # the push bar's rear face and mid height


def _api():
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    return Gf, Sdf, UsdGeom, UsdPhysics, UsdShade


def _schema(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    kinds = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float, str: Sdf.ValueTypeNames.Token}
    for k, v in attrs.items():
        prim.CreateAttribute(k, kinds[type(v)]).Set(v)


def _look(stage, path, rgb):
    Gf, Sdf, _, _, UsdShade = _api()
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _surface(stage, path, mu, combine=None):
    _, _, _, UsdPhysics, UsdShade = _api()
    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(mu)
    api.CreateDynamicFrictionAttr(mu)
    api.CreateRestitutionAttr(0.0)
    if combine:
        _schema(m.GetPrim(), "PhysxMaterialAPI", **{"physxMaterial:frictionCombineMode": combine})
    return m


def _collide(prim, phys):
    _, _, _, UsdPhysics, UsdShade = _api()
    UsdPhysics.CollisionAPI.Apply(prim)
    _schema(prim, "PhysxCollisionAPI", **{"physxCollision:contactOffset": 0.002, "physxCollision:restOffset": 0.0})
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(phys, UsdShade.Tokens.weakerThanDescendants, "physics")


def _body(prim, mass, com=None):
    Gf, _, _, UsdPhysics, _ = _api()
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)
    _schema(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": ITERS,
                                          "physxRigidBody:solverVelocityIterationCount": VEL_ITERS,
                                          "physxRigidBody:maxDepenetrationVelocity": 1.0})
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(float(mass))
    if com is not None:
        m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))


def build(stage, path):
    Gf, _, UsdGeom, UsdPhysics, UsdShade = _api()
    root = stage.GetPrimAtPath(path)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    _schema(root, "PhysxArticulationAPI", **{"physxArticulation:enabledSelfCollisions": False,
                                             "physxArticulation:solverPositionIterationCount": ITERS,
                                             "physxArticulation:solverVelocityIterationCount": VEL_ITERS})
    looks = {k: _look(stage, f"{path}/Looks/{k}", c) for k, c in COLORS.items()}
    grip = _surface(stage, f"{path}/Physics/tyre", MU["wheel"], "max")
    body = _surface(stage, f"{path}/Physics/body", MU["body"])
    chassis = UsdGeom.Xform.Define(stage, f"{path}/chassis").GetPrim()
    _body(chassis, CHASSIS_MASS, COM)
    for name, (lo, hi, look) in BOXES.items():
        c = UsdGeom.Cube.Define(stage, f"{path}/chassis/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*[(a + b) / 2 for a, b in zip(lo, hi)]))
        c.AddScaleOp().Set(Gf.Vec3f(*[b - a for a, b in zip(lo, hi)]))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(looks[look])
        _collide(c.GetPrim(), body)
    wheels = [("rear_l", 0.0, REAR["y"], REAR), ("rear_r", 0.0, -REAR["y"], REAR),
              ("caster_l", CASTER["x"], CASTER["y"], CASTER), ("caster_r", CASTER["x"], -CASTER["y"], CASTER)]
    for name, x, y, w in wheels:
        prim = UsdGeom.Xform.Define(stage, f"{path}/{name}").GetPrim()
        _body(prim, w["mass"])
        at = Gf.Vec3d(float(x), float(y), float(w["r"]))
        s = UsdGeom.Sphere.Define(stage, f"{path}/{name}/collide")
        s.CreateRadiusAttr(float(w["r"]))
        s.AddTranslateOp().Set(at)
        s.CreatePurposeAttr("guide")
        _collide(s.GetPrim(), grip)
        v = UsdGeom.Cylinder.Define(stage, f"{path}/{name}/tyre")
        v.CreateRadiusAttr(float(w["r"]))
        v.CreateHeightAttr(float(w["w"]))
        v.CreateAxisAttr("Y")
        v.AddTranslateOp().Set(at)
        UsdShade.MaterialBindingAPI.Apply(v.GetPrim()).Bind(looks["tyre"])
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/spin_{name}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([chassis.GetPath()])
        j.CreateBody1Rel().SetTargets([prim.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, at)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, at)))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0002)
    return root


SEAT = dict(x0=0.0, x1=0.100, half_y=0.060, top=0.075)
# the whole chair's outline in its own frame, for "hands off the chair"
OUTLINE = ((-0.115, -0.115, 0.0), (0.155, 0.115, 0.165))

"""A 1:6 two-position bus bike rack, lowered, as one articulation: a rigid 6 kg frame (two U-channel wheel trays, the
spine at the bus end with the hinge posts, mounting bars) and two support arms, each on a revolute hinge about the
rack's y axis at the bus end. Each arm is a black bar with a black cross-knob and a yellow hook at its end; the hook
is a pad with a lip either side of the tyre. Stowed (hinge angle 0) an arm leans 15 degrees back toward the bus; swung
out (negative angle) it comes over and down onto a tyre in its tray.

Geometry is given in table coordinates (layout.py) and converted to the rack frame (origin layout.RACK_ORIGIN) here.
The pure functions need only NumPy; build() needs Kit's pxr and is called from run.py."""

import math

import numpy as np

import bike as B
import layout as L

FRAME_MASS = 6.0
ARM_MASS = 0.08  # the hook's weight is what holds it on the tyre (a real arm has a spring)
ARM_DAMPING = 0.30  # N m s / rad on each hinge (an Isaac Lab implicit actuator with no stiffness)
STOW = math.radians(15.0)  # stowed lean back toward the bus, from vertical
SIGN = 1.0  # +1 if PhysX's revolute angle about +y is right-handed (tilts +z toward +x). UNVERIFIED: check the trace
BAR_W = 0.008
PAD = (0.040, 0.014, 0.006)  # hook pad: along x (across the tyre), along y, thickness
LIP = dict(x=0.019, w=0.006, depth=0.008)  # lip centres either side of the pad centre, width, depth below the pad
KNOB = dict(at=0.090, length=0.100, w=0.010)  # along the bar from the hinge; sticks out sideways (away from the bike)
LOW_STOP_Z = 0.030  # with no bike, an arm swings down until its lips are this high
GREY, BLACK, YELLOW = (0.45, 0.46, 0.48), (0.08, 0.08, 0.09), (0.95, 0.78, 0.05)
FRICTION = 0.8


def ry(a):
    """Rotation about +y by a (right-handed: +z tilts toward +x)."""
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def to_rack(p):
    return np.asarray(p, float) - np.asarray(L.RACK_ORIGIN, float)


def _arm_rest(name):
    """The arm resting on the front tyre of a bike centred in its tray, in table coordinates."""
    side = L.HOOK_END[name]
    y = L.ARM_Y[name]
    x = L.TRAYS[name]
    dy = L.FRONT_OFFSET - PAD[1] / 2  # the pad's edge nearest the top of the tyre
    z = B.R + math.sqrt(B.R ** 2 - dy ** 2)  # pad bottom on the tyre
    P = np.array((L.PIVOT_X, y, L.PIVOT_Z))
    H = np.array((x, y, z))  # pad bottom centre: the point that rests on the tyre
    A = np.array((x + PAD[0] / 2, y, z + PAD[2]))  # where the bar meets the pad (its bus-side top edge)
    d = A - P
    phi = math.atan2(d[0], d[2])  # bar direction, from vertical toward +x
    return dict(side=side, P=P, H=H, A=A, phi=phi, length=float(np.linalg.norm(d)), q_rest=phi - STOW)


def arm_shapes(name):
    """The arm's boxes at rest, table coordinates: (centre, size, colour, rotation about y)."""
    a = _arm_rest(name)
    P, H, A, phi, side = a["P"], a["H"], a["A"], a["phi"], a["side"]
    u = (A - P) / a["length"]
    knob_c = P + u * KNOB["at"] + np.array((0.0, side * (BAR_W / 2 + KNOB["length"] / 2), 0.0))
    return [
        ((P + A) / 2, (BAR_W, BAR_W, a["length"]), BLACK, phi),
        (H + np.array((0.0, 0.0, PAD[2] / 2)), PAD, YELLOW, 0.0),
        (H + np.array((LIP["x"], 0.0, -LIP["depth"] / 2)), (LIP["w"], PAD[1], LIP["depth"]), YELLOW, 0.0),
        (H + np.array((-LIP["x"], 0.0, -LIP["depth"] / 2)), (LIP["w"], PAD[1], LIP["depth"]), YELLOW, 0.0),
        (knob_c, (KNOB["w"], KNOB["length"], KNOB["w"]), BLACK, phi),
    ]


def arm_local(name):
    """The arm's boxes in its link frame (origin at the hinge, axes of the rack at hinge angle 0)."""
    a = _arm_rest(name)
    back = ry(-a["q_rest"])
    return [(back @ (np.asarray(c) - a["P"]), s, col, rot - a["q_rest"]) for c, s, col, rot in arm_shapes(name)]


def q_rest(name):
    """The hinge angle with the hook resting on a centred bike's front tyre (rad, negative = swung out)."""
    return _arm_rest(name)["q_rest"]


def point_at(name, local_rest_point, q):
    """A point fixed to the arm, given where it is at rest (table coordinates), at hinge angle q (table coordinates)."""
    a = _arm_rest(name)
    p0 = ry(-a["q_rest"]) @ (np.asarray(local_rest_point, float) - a["P"])
    return a["P"] + ry(q) @ p0


def hook(name, q):
    """The hook pad's bottom centre at hinge angle q, table coordinates."""
    return point_at(name, _arm_rest(name)["H"], q)


def lips_bottom(name, q):
    return point_at(name, _arm_rest(name)["H"] + np.array((0.0, 0.0, -LIP["depth"])), q)


def knob(name, q):
    """The cross-knob's centre at hinge angle q, table coordinates."""
    c = arm_shapes(name)[4][0]
    return point_at(name, c, q)


def q_low(name):
    """The hinge's swung-out limit: where the lips come down to LOW_STOP_Z with no bike in the tray."""
    lo, hi = q_rest(name) - math.radians(80), q_rest(name)
    for _ in range(60):
        mid = (lo + hi) / 2
        if lips_bottom(name, mid)[2] > LOW_STOP_Z:
            hi = mid
        else:
            lo = mid
    return hi


def frame_shapes():
    """The rack frame's boxes, table coordinates: (centre, size, colour)."""
    t, out = L.TRAY, []
    yc, ylen = (t["y0"] + t["y1"]) / 2, t["y1"] - t["y0"]
    for x in L.TRAYS.values():
        for s in (-1, 1):  # the U-channel's side walls
            out.append(((x + s * (t["half_in"] + t["wall"] / 2), yc, t["h"] / 2), (t["wall"], ylen, t["h"]), GREY))
        for y in (t["y0"] - t["wall"] / 2, t["y1"] + t["wall"] / 2):  # end stops
            out.append(((x, y, t["h"] / 2), (2 * t["half_in"] + 2 * t["wall"], t["wall"], t["h"]), GREY))
    xs = sorted(L.TRAYS.values())
    sp = L.SPINE
    for y in (t["y0"] - 0.012, t["y1"] + 0.012):  # cross members joining the trays to the spine
        out.append((((xs[0] - t["half_in"] + sp["x"]) / 2, y, 0.004), (sp["x"] - xs[0] + t["half_in"], 0.012, 0.008), GREY))
    out.append(((sp["x"], (sp["y0"] + sp["y1"]) / 2, sp["h"] / 2), (2 * sp["half_x"], sp["y1"] - sp["y0"], sp["h"]), GREY))
    for name, y in L.ARM_Y.items():  # hinge posts beside each arm, on the bike side of it
        py = y - L.HOOK_END[name] * (BAR_W / 2 + 0.006)
        out.append(((L.PIVOT_X, py, (L.PIVOT_Z + 0.008) / 2), (0.014, 0.008, L.PIVOT_Z + 0.008), BLACK))
    for y in (-0.15, 0.12):  # mounting bars toward the bumper
        out.append((((sp["x"] + L.BUMPER["x0"] - 0.002) / 2, y, 0.012), (L.BUMPER["x0"] - 0.002 - sp["x"], 0.014, 0.010), GREY))
    return out


# -- USD / PhysX (Kit only) ---------------------------------------------------------------------------------------------


def _schema(prim, schema, **attrs):
    from pxr import Sdf

    if not prim.ApplyAPI(schema):
        prim.AddAppliedSchema(schema)
    kinds = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float}
    for k, v in attrs.items():
        prim.CreateAttribute(k, kinds[type(v)]).Set(v)


def build(stage, path):
    """Author the rack under `path` (an Xform already placed at the rack's pose)."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    looks = {}

    def look(rgb):
        if rgb not in looks:
            mat = UsdShade.Material.Define(stage, f"{path}/Looks/c{len(looks)}")
            sh = UsdShade.Shader.Define(stage, f"{path}/Looks/c{len(looks)}/shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[c ** 2.2 for c in rgb]))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            looks[rgb] = mat
        return looks[rgb]

    phys = UsdShade.Material.Define(stage, f"{path}/Looks/physics")
    m = UsdPhysics.MaterialAPI.Apply(phys.GetPrim())
    m.CreateStaticFrictionAttr(FRICTION)
    m.CreateDynamicFrictionAttr(FRICTION)
    m.CreateRestitutionAttr(0.0)

    def body(p, mass):
        x = UsdGeom.Xform.Define(stage, p)
        prim = x.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        _schema(prim, "PhysxRigidBodyAPI", **{"physxRigidBody:solverPositionIterationCount": 32,
                                              "physxRigidBody:solverVelocityIterationCount": 4,
                                              "physxRigidBody:maxDepenetrationVelocity": 1.0})
        return x

    def box(p, centre, size, color, angle_y=0.0):
        c = UsdGeom.Cube.Define(stage, p)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if angle_y:
            h = angle_y / 2
            c.AddOrientOp().Set(Gf.Quatf(math.cos(h), Gf.Vec3f(0.0, math.sin(h), 0.0)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(look(color))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        _schema(c.GetPrim(), "PhysxCollisionAPI", **{"physxCollision:contactOffset": 0.002, "physxCollision:restOffset": 0.0})
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(phys, UsdShade.Tokens.weakerThanDescendants, "physics")

    frame = body(f"{path}/frame", FRAME_MASS)
    UsdPhysics.ArticulationRootAPI.Apply(frame.GetPrim())
    _schema(frame.GetPrim(), "PhysxArticulationAPI", **{"physxArticulation:enabledSelfCollisions": False,
                                                        "physxArticulation:solverPositionIterationCount": 32,
                                                        "physxArticulation:solverVelocityIterationCount": 4})
    for i, (c, s, col) in enumerate(frame_shapes()):
        box(f"{path}/frame/box_{i}", to_rack(c), s, col)
    for name in L.ARM_Y:
        a = _arm_rest(name)
        pivot = to_rack(a["P"])
        link = body(f"{path}/arm_{name}", ARM_MASS)
        link.AddTranslateOp().Set(Gf.Vec3d(*map(float, pivot)))
        for i, (c, s, col, rot) in enumerate(arm_local(name)):
            box(f"{path}/arm_{name}/box_{i}", c, s, col, rot)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/arm_{name}_hinge")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([frame.GetPath()])
        j.CreateBody1Rel().SetTargets([link.GetPath()])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pivot)))
        j.CreateLocalRot0Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        lo, hi = sorted((0.0, SIGN * math.degrees(q_low(name))))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
    return stage.GetPrimAtPath(path)


JOINTS = {name: f"arm_{name}_hinge" for name in L.ARM_Y}

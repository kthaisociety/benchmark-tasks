"""A wardrobe rail is refitted between the two side panels of a wardrobe, under the hat shelf, as rails are mounted
in real wardrobes: the shelf sits 5.5 cm above the rail, so the rail and its supports are seen and reached only from
below and in front. The left end goes into a closed socket, the right end into an open U socket. The rail is longer
than the gap minus the closed socket's depth, and the shelf leaves no room to tilt it steeply, so it goes in nearly
level and swung in plan: the left end is fed into the closed socket with the right end swung out toward the front,
then the right end is swung back, lifted just over the U's front cheek, and lowered into the U. Both arms hold the
rail from the front, fingers closing on its top and bottom.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import env, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Fit the wardrobe rail into its two end supports."

WHITE, CHROME, GREY, FOAM = (0.92, 0.91, 0.88), (0.78, 0.80, 0.83), (0.35, 0.36, 0.38), (0.25, 0.45, 0.80)
AX = 0.32             # x of the socket axis (m)
ZS = 0.08             # socket axis height above the wardrobe base's underside (m)
GAP = 0.300           # inner face of each side panel, |y| (m)
L, D = 0.592, 0.025   # round chrome wardrobe rod: length, diameter (m)
H = W = D
CUP = 0.010           # closed socket depth (m): lip at y = GAP - CUP
U_DEPTH = 0.015       # open socket depth along y (m)
BX, BZ, WALL = 0.029, 0.029, 0.004   # closed socket bore (round socket modelled square): width (x), height (z); wall
SHELF_GAP = float(os.environ.get("RAIL_SHELF_GAP", 0.055))   # rail top to the hat shelf's underside (m)
SHELF_T = 0.018       # hat shelf thickness (m)
SHELF_X0 = float(os.environ.get("RAIL_SHELF_X0", -0.030))   # hat shelf front edge, relative to the rail axis (m):
#   3.0 cm in front of it: the rod's front is at -1.25 cm and the supports' front walls at -1.85 cm, so from
#   directly above the shelf covers the rod and both supports with 1.15 cm to spare
SHELF_Z = ZS + H / 2 + SHELF_GAP + SHELF_T / 2

_lip = GAP - CUP / 2
WARDROBE = part("wardrobe", pos=(AX, 0.0, TOP), mass=25.0, friction=1.2, color=WHITE, boxes=[
    ((0, 0, 0.009), (0.20, 2 * GAP + 0.036, 0.018), WHITE),                 # base board
    ((0, GAP + 0.009, 0.089), (0.20, 0.018, 0.16), WHITE),                  # left side panel
    ((0, -GAP - 0.009, 0.089), (0.20, 0.018, 0.16), WHITE),                 # right side panel
    # hat shelf between the panels, 5.5 cm above the rail, from just in front of the rail to the back
    (((SHELF_X0 + 0.10) / 2, 0, SHELF_Z), (0.10 - SHELF_X0, 2 * GAP, SHELF_T), WHITE),
    # closed end support on the left panel: an oval cup, here a 21 x 38 mm bore, 10 mm deep
    ((0, _lip, ZS + BZ / 2 + WALL / 2), (BX + 2 * WALL, CUP, WALL), CHROME),
    ((0, _lip, ZS - BZ / 2 - WALL / 2), (BX + 2 * WALL, CUP, WALL), CHROME),
    ((-BX / 2 - WALL / 2, _lip, ZS), (WALL, CUP, BZ), CHROME),
    ((BX / 2 + WALL / 2, _lip, ZS), (WALL, CUP, BZ), CHROME),
    # open end support on the right panel: floor at the rail's underside, two cheeks, open on top and toward the rail
    ((0, -GAP + U_DEPTH / 2, ZS - H / 2 - WALL / 2), (BX + 2 * WALL, U_DEPTH, WALL), CHROME),
    ((-BX / 2 - WALL / 2, -GAP + U_DEPTH / 2, ZS - H / 2 + 0.010), (WALL, U_DEPTH, 0.020), CHROME),
    ((BX / 2 + WALL / 2, -GAP + U_DEPTH / 2, ZS - H / 2 + 0.010), (WALL, U_DEPTH, 0.020), CHROME)])
RAIL0 = (0.16, 0.0, TOP + 0.05 + H / 2 + 0.001)
# the rod is built along the part's z axis (partkit cylinders are vertical) and laid along y at every reset
RAIL = part("rail", pos=(0.16, 0.0, TOP + 0.45), mass=0.35, friction=1.2, color=CHROME,
            boxes=[((0, 0, 0), (0.012, 0.012, L - 0.02), CHROME)], cylinders=[((0, 0, 0), D / 2, L, CHROME)])
RAIL_Q = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))   # -90 degrees about x: the rod's axis (part z) along +y
OBJECTS = [WARDROBE, RAIL,
           part("foam", pos=(0.16, 0.0, TOP), mass=3.0, friction=1.2, color=FOAM, boxes=[   # two grooved foam blocks
               ((0, 0.10, 0.025), (0.06, 0.06, 0.05), FOAM), ((0, -0.10, 0.025), (0.06, 0.06, 0.05), FOAM),
               ((-D / 2 - 0.006, 0.10, 0.055), (0.012, 0.06, 0.010), FOAM), ((D / 2 + 0.006, 0.10, 0.055), (0.012, 0.06, 0.010), FOAM),
               ((-D / 2 - 0.006, -0.10, 0.055), (0.012, 0.06, 0.010), FOAM), ((D / 2 + 0.006, -0.10, 0.055), (0.012, 0.06, 0.010), FOAM)])]

_S: dict = {}


@on_reset
def _reset(environment=None):
    if environment is not None:
        write_pose(environment, "rail", RAIL0, RAIL_Q)
    _S.update(recent=[], step=0, scored=None, last=0.0)


@on_step
def _count(environment=None):
    _S["step"] += 1


_reset()


def _arm_clear(rail) -> bool:
    """Both arms: fingers at least 90% open and both pad tips at least 3 cm from the rail's axis."""
    e = env()
    if e is None:
        return False
    scene = e.unwrapped.scene
    a0, a1 = rail.to_world((0, 0, -L / 2)), rail.to_world((0, 0, L / 2))
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        q = np.asarray(getattr(arm.data.joint_pos, "torch", arm.data.joint_pos)[0].detach().cpu(), float)
        fingers = q[arm.find_joints(["joint7", "joint8"], preserve_order=True)[0]]
        if fingers.mean() / FINGER_OPEN < 0.9:
            return False
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        for t in tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - org:
            u = np.clip(np.dot(t - a0, a1 - a0) / np.dot(a1 - a0, a1 - a0), 0, 1)
            if np.linalg.norm(t - (a0 + u * (a1 - a0))) < 0.03:
                return False
    return True


def _arm_dbg():
    e = env()
    scene = e.unwrapped.scene
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    out = []
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        q = np.asarray(getattr(arm.data.joint_pos, "torch", arm.data.joint_pos)[0].detach().cpu(), float)
        f = q[arm.find_joints(["joint7", "joint8"], preserve_order=True)[0]]
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        t = tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - org
        out.append(f"{name[0]} f={np.round(f*1000,1)} tips={np.round(t*1000,0).tolist()}")
    return " ".join(out)


def _state(objects):
    w, r = objects["wardrobe"], objects["rail"]
    left = w.to_local(r.to_world((0, 0, L / 2)))     # rail end centres in the wardrobe frame (m)
    right = w.to_local(r.to_world((0, 0, -L / 2)))
    axis = right - left
    tilt = math.degrees(math.asin(abs(axis[2]) / np.linalg.norm(axis)))
    left_in = (left[1] >= GAP - CUP + 0.008 and abs(left[0]) <= (BX - W) / 2 + 0.001
               and abs(left[2] - ZS) <= (BZ - H) / 2 + 0.001)     # end centre inside the bore, 8 mm or more deep
    right_in = (right[1] <= -GAP + U_DEPTH - 0.005 and abs(right[0]) <= BX / 2 and abs(right[2] - ZS) <= 0.003)
    return left, right, tilt, left_in, right_in


def score(objects) -> float:
    """End state only. 0.4 if the left end is at least 8 mm into the closed socket's bore; 0.4 if the
    right end rests on the U socket's floor (height within 3 mm, at least 5 mm over it); 0.2 if the rail is level
    within 1 degree with both ends in. Credit only if both arms have let go (fingers open, pads 3 cm from the rail)
    and the rail has been still (moved under 1 mm) for the last second."""
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    left, right, tilt, left_in, right_in = _state(objects)
    pos = np.asarray(objects["rail"].pos, float)
    recent = _S["recent"]
    recent.append((pos.copy(), _arm_clear(objects["rail"])))
    del recent[:-20]
    still = len(recent) == 20 and all(abs(p - pos).max() < 0.001 and c for p, c in recent)
    s = 0.4 * left_in + 0.4 * right_in + (0.2 if left_in and right_in and tilt <= 1.0 else 0.0)
    _S["last"] = s if still else 0.0
    if os.environ.get("RAIL_DEBUG") and _S["step"] % 20 == 0:
        print(f"[dbg] step {_S['step']} left {np.round(left*1000,1)} right {np.round(right*1000,1)} tilt {tilt:.1f} "
              f"in {left_in},{right_in} still {still} score {_S['last']:.2f} | {_arm_dbg()}", flush=True)
    return _S["last"]


# -- scripted demo: both arms hold the rail from the front, gripper pointing forward (+x) and fingers closing on the
# rail's top and bottom, the only way to reach it under the shelf. They lift it off the foam blocks, swing it so the
# right end is toward the front, feed the left end into the closed socket, swing the right end back while lifting it
# just over the U's front cheek, lower it into the U, let go and back out. ----------------------------------------------
from yam_arena import demo as _ydemo  # noqa: E402

if not getattr(_ydemo, "_wr_matrix_grips", False):   # let a Waypoint's yaw be a full 3x3 TCP rotation (world frame)
    _plain_grip = _ydemo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else _plain_grip(yaw)

    _ydemo.grip_rotation = _grip_rotation
    _ydemo._wr_matrix_grips = True
_R0 = np.asarray(_ydemo.grip_rotation(0.0), float)    # straight-down grip: approach -z, fingers closing along y

GRIP = float(os.environ.get("RAIL_GRIP", 0.18))      # grasp points, m either side of the rail centre
FLIP = np.diag([-1.0, -1.0, 1.0])   # the right wrist reaches the same grip turned half a turn
PITCH = math.radians(float(os.environ.get("RAIL_PITCH", 30.0)))   # grip tipped down from horizontal


def _side_R(psi, theta):
    """TCP rotation for a grip from the front on a rail swung psi in plan (right end toward the front) and tilted theta
    (right end up): approach horizontal and square to the rail, fingers closing along the rail's own up direction."""
    e = np.array([-math.sin(psi) * math.cos(theta), -math.cos(psi) * math.cos(theta), math.sin(theta)])  # left->right
    ah = np.array([math.cos(psi), -math.sin(psi), 0.0])
    n = np.cross(e, ah)
    n /= np.linalg.norm(n)                       # the rail's up direction
    a = math.cos(PITCH) * ah - math.sin(PITCH) * n   # approach: forward, tipped down by PITCH
    c = np.cross(e, a)
    c /= np.linalg.norm(c)
    rot = np.column_stack([np.cross(a, c), c, -a])   # world -z -> a, world y -> c
    return rot @ _R0


def _grips(lx, ly, lz, psi_deg, theta_deg):
    """TCP targets and rotations for both arms with the rail's left end centre at (lx, ly, lz)."""
    psi, th = math.radians(psi_deg), math.radians(theta_deg)
    e = np.array([-math.sin(psi) * math.cos(th), -math.cos(psi) * math.cos(th), math.sin(th)])
    p = np.array([lx, ly, lz])
    gl, gr = p + (L / 2 - GRIP) * e, p + (L / 2 + GRIP) * e
    R = _side_R(psi, th)
    return tuple(gl.tolist()), tuple(gr.tolist()), R


def demo():
    v = os.environ.get("RAIL_VARIANT", "")
    zs = TOP + ZS
    PSI = float(os.environ.get("RAIL_PSI", 18.0))     # swing in plan while feeding the left end (deg)
    LIFT = float(os.environ.get("RAIL_LIFT", 3.6))    # right-end tilt over the U cheek (deg): ~3.7 cm at the end
    YB = float(os.environ.get("RAIL_YB", 0.004))      # aim the left end this much further left (it lags in the grip)
    DZ = float(os.environ.get("RAIL_DZ", 0.0))        # aim offset for the left end's height
    x0, z0 = RAIL0[0], RAIL0[2]
    Y0 = float(os.environ.get("RAIL_Y0", 0.270))   # left end y while carried in, clear of the left panel
    zl = zs - DZ
    keys = [  # (duration, left end x, y, z, psi deg, theta deg, finger, label)
        (2.0, x0 - 0.08, L / 2, z0, 0, 0, FINGER_OPEN, "in front of the rail"),
        (1.5, x0, L / 2, z0, 0, 0, FINGER_OPEN, "fingers around it"),
        (0.8, x0, L / 2, z0, 0, 0, FINGER_CLOSED, "pinch top and bottom"),
        (1.5, x0, L / 2, z0 + 0.03, 0, 0, FINGER_CLOSED, "lift off the foam"),
        (2.5, AX - 0.05, Y0, zl, PSI, LIFT, FINGER_CLOSED, "swing right end to the front"),
        (2.0, AX, Y0, zl, PSI, LIFT, FINGER_CLOSED, "left end at the socket"),
        (1.5, AX, 0.298 + YB, zl, PSI, LIFT, FINGER_CLOSED, "feed the left end in"),
        (1.5, AX, 0.299 + YB, zl, PSI * 0.6, LIFT, FINGER_CLOSED, "swing back"),
        (1.5, AX, 0.2995 + YB, zl, PSI * 0.25, LIFT, FINGER_CLOSED, "swing back, over the cheek"),
        (1.5, AX, 0.2995 + YB, zl, 0, LIFT, FINGER_CLOSED, "right end over the U"),
        (1.5, AX, 0.2995 + YB, zs, 0, 0, FINGER_CLOSED, "lower into the U"),
        (0.8, AX, 0.2995 + YB, zs, 0, 0, FINGER_CLOSED, "settle"),
    ]
    if v == "tilt_in":   # wrong: the old way, right end raised 13 degrees: the shelf is in the way
        keys = keys[:4] + [(2.5, AX - 0.05, 0.286, zl, 0, 13.0, FINGER_CLOSED, "tilt, right end up"),
                           (2.0, AX, 0.286, zl, 0, 13.0, FINGER_CLOSED, "carry in"),
                           (1.5, AX, 0.2995 + YB, zl, 0, 13.0, FINGER_CLOSED, "feed"),
                           (2.0, AX, 0.2995 + YB, zs, 0, 0, FINGER_CLOSED, "swing down")]
    if v == "level":     # wrong: carry it in level and square, push toward the closed socket
        keys = keys[:4] + [(2.5, AX - 0.05, 0.286, zs, 0, 0, FINGER_CLOSED, "level"),
                           (2.0, AX, 0.286, zs, 0, 0, FINGER_CLOSED, "carry in"),
                           (2.0, AX, 0.2995 + YB, zs, 0, 0, FINGER_CLOSED, "push left")]
    if v == "nothing":
        return {"left": [Waypoint(3.0, (0.18, 0.36, TOP + 0.22), FINGER_OPEN, "park")],
                "right": [Waypoint(3.0, (0.18, -0.36, TOP + 0.22), FINGER_OPEN, "park")]}
    if v == "hold":
        keys = keys + [(4.0, AX, 0.2995 + YB, zs, 0, 0, FINGER_CLOSED, "hold")]
    left, right = [], []
    XB = float(os.environ.get("RAIL_XB", 0.009))   # the rail sits this far toward -x of where the grip aims (m)
    ZB = float(os.environ.get("RAIL_ZB", 0.014))   # ... and this far low
    XBR = float(os.environ.get("RAIL_XBR", 0.014))  # extra for the right grip
    for i, (dur, lx, ly, lz, psi, th, f, lab) in enumerate(keys):
        gl, gr, R = _grips(lx, ly, lz, psi, th)
        if i >= 4:   # once carried: aim off by the rail's offset in the grip
            gl, gr = (gl[0] + XB, gl[1], gl[2] + ZB), (gr[0] + XB + XBR, gr[1], gr[2] + ZB)
        left.append(Waypoint(dur, gl, f, lab, R))
        right.append(Waypoint(dur, gr, f, lab, R @ FLIP))   # same grip turned 180 degrees about the approach
    if v != "hold":
        for arm, side in ((left, 1), (right, -1)):
            p, R = arm[-1].pos, arm[-1].yaw
            arm += [Waypoint(0.8, p, FINGER_OPEN, "let go", R),
                    Waypoint(2.5, (0.15, p[1], p[2] - 0.02), FINGER_OPEN, "back out low from under the shelf", R),
                    Waypoint(1.5, (0.15, p[1], TOP + 0.22), FINGER_OPEN, "rise in front of the shelf", R),
                    Waypoint(2.0, (0.18, 0.36 * side, TOP + 0.22), FINGER_OPEN, "park"),
                    Waypoint(2.0, (0.18, 0.36 * side, TOP + 0.22), FINGER_OPEN, "hold")]
    return {"left": left, "right": right}

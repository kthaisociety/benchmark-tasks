"""On the station table, in front of the right arm, stands a full 275 ml long-neck glass bottle closed with a red crown
cap on a standard 26 mm crown finish. Further forward a Starr X wall-mount bottle opener (Brown Mfg. Co.) is screwed to
the front of a dark wooden post, its mouth facing down and toward the robot: inside the mouth a fulcrum pad presses on
the middle of a cap's top and a prying lip catches under the edge of its skirt. A round cork coaster lies on the table
near the middle.

A crown cap is crimped under the glass bead and cannot be pulled or twisted off by hand. To open the bottle on the
opener, the bottle is tipped well over, its top toward the post, and its cap pushed up into the mouth until its top
meets the pad; then it is moved so the lip hooks under the skirt's edge, and the bottle is pressed down toward upright:
it pivots on the pad and the lip pries the crimp open (about 0.9 N m on the cap, as for a real pry-off crown) until it
lets go. The open bottle is then stood upright on the coaster and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import _SPECS, _np, _part_state, on_reset, on_step, part, set_joint_enabled, tilt_joint, write_pose  # noqa: E402,I001
from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY, SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import DOWN_GRIP, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

GLASS = (0.16, 0.46, 0.24)
LABEL = (0.93, 0.92, 0.86)
CAP_RED = (0.80, 0.10, 0.08)
STEEL = (0.70, 0.71, 0.73)  # zinc-plated cast iron
SCREW = (0.30, 0.30, 0.32)
POST = (0.18, 0.19, 0.21)  # dark-stained hardwood
CORK = (0.72, 0.55, 0.36)

# -- the bottle: a 275 ml long-neck with a 26 mm crown finish; origin at the middle of its base ------------------------
BODY_R, BODY_H = 0.029, 0.110  # 58 mm body
NECK_GRIP_R, NECK_GRIP_Z0, NECK_GRIP_Z1 = 0.016, 0.130, 0.162  # the lower neck, 32 mm across
NECK_R = 0.01175  # 23.5 mm: the neck just under the finish
BEAD_R, BEAD_H = 0.0132, 0.0043  # 26.4 mm: the bead (transfer ring) the cap is crimped under
SEAT = 0.185  # top of the finish, where the cap's liner sits (above the base)
BOTTLE_MASS = 0.45  # full
BOTTLE_AT = (0.27, -0.25)

# -- the crown cap, crimped: 28.8 mm across, 6 mm tall; origin at its underside's centre (on the finish) ---------------
CAP_R = 0.0144  # outside of the crimped skirt
SKIRT_T = 0.0010  # the skirt ring's wall in the model: 13.4 to 14.4 mm from the axis
SKIRT_H = 0.0050  # skirt below the crown's top
TOP_T = 0.0010  # the crown's top
CAP_H = SKIRT_H + TOP_T
FLUTES = 12  # the skirt is modelled as 12 flat segments
CAP_MASS = 0.003

# -- the crimp: a joint holding the cap on the finish (see README "The crimp") ---------------------------------------
# The cap can't slide or turn on the finish; it can only tilt off it, against a spring, about the middle of the finish's
# top. Pried open, the cap lets go once the moment on it reaches T_OPEN: by then the side of the skirt being lifted has
# come up POP_LIFT off the bead. T_OPEN is a 26 mm pry-off crown's opening torque (0.45 to 1.0 N m in a crown maker's
# data sheet, about 0.9 N m in a patent's tests of 21-flute pry-off crowns).
T_OPEN = 0.9  # N m
POP_LIFT = 0.0015  # m
THETA_POP = POP_LIFT / CAP_R  # rad: the cap's tilt on the finish when it lets go (6.0 degrees)
CRIMP_K = T_OPEN / THETA_POP  # N m/rad: the crimp's stiffness against tilting (8.6)
CRIMP_DAMPING = 0.005  # N m s/rad

# -- the opener: a Starr X wall-mount bottle opener (Brown Mfg. Co.), screwed to the front of a post ------------------
# Its working parts follow the maker's patents (US 1,534,211 and US 2,333,088): a fulcrum pad that presses on the middle
# of the cap's top, and a ledge whose upturned edge, the prying lip, catches under the edge of the cap's skirt, with the
# edge of the mouth round the cap (the centring member) and back stops above the lip. As in the patent's Fig. 2, the bottle
# goes in tipped TILT_IN from upright, its top toward the wall. The jaw is laid out in the "jaw frame" J: the bottle's
# frame as it sits in the mouth, with its origin on the middle of the cap's top (on the pad's nose), z along the bottle's
# axis toward the cap, x toward the wall (and down), y as the world's.
TILT_IN = math.radians(53.0)
PAD_AT = (0.40, -0.20, TOP + 0.176)  # world: the middle of the cap's top on the pad's nose, cap in the mouth
JAW_Y = PAD_AT[1]
WALL_X = PAD_AT[0] + 0.015  # the post's front face, where the opener's back is screwed on
JAW_GAP = 0.001  # the lip's top this far beyond the skirt's edge (along the axis) with the cap's top on the pad
LIP_TIP_J = -(CAP_R - SKIRT_T + 0.0001)  # the lip's edge, 13.5 mm from the cap's axis: 0.9 mm under the skirt's wall
CENTRE_J = -(CAP_R + 0.0005)  # the mouth's edge round the cap, 0.5 mm off the skirt, on the lip's side
NOSE_HALF = 0.003  # the pad's nose: 6 mm wide over the middle of the cap


def _rot_y(a: float) -> np.ndarray:
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])


R_J = _rot_y(TILT_IN)


def jaw_to_world(p_j) -> np.ndarray:
    return np.asarray(PAD_AT, dtype=np.float64) + R_J @ np.asarray(p_j, dtype=np.float64)


def _wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = ik.matrix_to_quat_xyzw(R)
    return (float(w), float(x), float(y), float(z))


def _jbox(x0, x1, y0, y1, z0, z1, color):
    """A box given by its extent in the jaw frame, as a box of the opener part (frame: world axes at PAD_AT)."""
    c = R_J @ np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return (tuple(c.tolist()), (x1 - x0, y1 - y0, z1 - z0), color, _wxyz(R_J))


def _wbox(x0, x1, y0, y1, z0, z1, color, R=None):
    """A box given by its extent along world axes, relative to PAD_AT (or turned R about its own middle)."""
    c = ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    return (c, (x1 - x0, y1 - y0, z1 - z0), color) + ((_wxyz(R),) if R is not None else ())


def _slab(a, b, width, thick, color):
    """A plate from point a to point b (x-z plane, relative to PAD_AT), ``width`` along y, ``thick`` across."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    ang = math.atan2(-(d[2]), d[0])  # turn about y taking +x onto a->b
    return (tuple(((a + b) / 2).tolist()), (float(np.linalg.norm(d)), width, thick), STEEL, _wxyz(_rot_y(ang)))


W_REL = WALL_X - PAD_AT[0]  # the wall, relative to PAD_AT
OPENER_BOXES = [
    # what touches the cap (US 2,333,088): the pad's nose, the prying lip and the mouth's edge round the cap, and the
    # back stops above the lip with room for the skirt to flare
    _jbox(-NOSE_HALF, NOSE_HALF, -0.006, 0.006, 0.0, 0.004, STEEL),  # fulcrum pad's nose (8)
    _jbox(NOSE_HALF, 0.022, -0.008, 0.008, 0.003, 0.008, STEEL),  # the pad, back to the lower base
    _jbox(-0.0200, LIP_TIP_J, -0.009, 0.009, -SKIRT_H - TOP_T - JAW_GAP - 0.002, -SKIRT_H - TOP_T - JAW_GAP,
          STEEL),  # ledge and prying lip (10, 11)
    _jbox(-0.0200, CENTRE_J, -0.012, 0.012, -SKIRT_H - TOP_T - JAW_GAP - 0.002, -0.0045, STEEL),  # mouth's edge (9)
    _jbox(-0.026, -0.0175, -0.015, 0.015, -0.0045, 0.012, STEEL),  # back stops and the apex (12)
    _jbox(-0.0200, 0.012, 0.0159, 0.0199, -0.009, 0.008, STEEL),  # the mouth's sides (the 1925 patent's arms)
    _jbox(-0.0200, 0.012, -0.0199, -0.0159, -0.009, 0.008, STEEL),
    # the cast body on the wall: lower base under the pad, upper base, and the wide upper face between them
    _wbox(W_REL - 0.0035, W_REL, -0.017, 0.017, -0.0365, -0.010, STEEL),
    _wbox(W_REL - 0.0035, W_REL, -0.013, 0.013, 0.020, 0.0465, STEEL),
    _slab((-0.0100, 0.0, 0.0300), (W_REL - 0.002, 0.0, 0.0440), 0.070, 0.004, STEEL),
]
# What the camera sees beyond the boxes above (no colliders, so the physics is the boxes alone): the cast body's rounded
# ears round the two screws and the hood's rounded nose over the mouth.
OPENER_LOOK_CYLS = [
    ((W_REL - 0.0020, 0.0, 0.0400), 0.0135, 0.004, STEEL, "X"),  # upper ear (the screw heads stand proud)
    ((W_REL - 0.0020, 0.0, -0.0270), 0.0180, 0.004, STEEL, "X"),  # lower ear
    (tuple((R_J @ np.array([-0.026, 0.0, -0.0045])).tolist()), 0.0045, 0.030, STEEL, "Y"),  # the hood's nose
]
OPENER = part("opener", pos=PAD_AT, mass=0.30, color=STEEL, friction=0.8, kinematic=True, boxes=OPENER_BOXES,
              cylinders=[((W_REL - 0.0045, 0.0, 0.0370), 0.004, 0.002, SCREW, "X"),  # the two screws
                         ((W_REL - 0.0045, 0.0, -0.0270), 0.004, 0.002, SCREW, "X")],
              visual_cylinders=OPENER_LOOK_CYLS)
POST_T, POST_W, POST_TOP = 0.025, 0.090, PAD_AT[2] + 0.060

# -- the coaster -------------------------------------------------------------------------------------------------------
COASTER = (0.31, -0.04)
COASTER_R, COASTER_H = 0.045, 0.004

# -- scoring -----------------------------------------------------------------------------------------------------------
TIPPED_DEG = 65.0  # the bottle tilted more than this, ever (the opener takes it tipped 53 degrees, closed)
UPRIGHT_DEG = 10.0
ON_COASTER = 0.030  # bottle axis within this of the coaster's centre
CATCH_IN = (0.0128, 0.0142)  # the lip's edge this far out from the cap's axis: under the skirt's wall
CATCH_SIDE = 0.006  # cap axis within this of the mouth's centre line sideways
CATCH_GAP = (-0.0005, 0.0025)  # the lip this far beyond the skirt's edge (in the cap's own frame)
CAUGHT_RECENT = 5  # the crimp's letting go counts as the opener's if the cap was caught within this many steps
LET_GO = 0.010  # let go: every part of both arms (fingers, gripper, wrist, arm) at least this far from the bottle
# Each moving link of a YAM arm and how finely its collision hulls (the convex hulls the simulator collides with) are
# sampled. Every point of a hull's surface is within 0.6 x that spacing of a sample, so a link only counts as clear
# when its nearest sample is that much further than LET_GO: the check never passes an arm closer than 1 cm.
ARM_LINKS = {"link1": 0.006, "link2": 0.006, "link3": 0.006, "link4": 0.006, "link5": 0.003, "gripper": 0.003,
             "tip_left": 0.003, "tip_right": 0.003}
ARMS = ("left_robot", "right_robot")
BASE_ROT = np.eye(3)  # both arm bases face +x (yam_arena's station rig); _link_poses checks it against the simulator
FK_TOL = 0.005  # m: each link's position from the kinematics must agree with the simulator's this closely


def _cyl(z0, z1, r, color):
    return ((0.0, 0.0, (z0 + z1) / 2), r, z1 - z0, color)


# the bottle's glass from its base up, (z0, z1, radius, colour): what it is built from and what the let-go check
# measures against (the core box inside the body is left out, it never reaches the surface)
BOTTLE_PROFILE = [
    (0.0, BODY_H, BODY_R, GLASS),
    (0.035, 0.085, BODY_R + 0.0004, LABEL),
    (BODY_H, 0.118, 0.025, GLASS),
    (0.118, 0.125, 0.020, GLASS),
    (0.125, NECK_GRIP_Z0, 0.017, GLASS),
    (NECK_GRIP_Z0, NECK_GRIP_Z1, NECK_GRIP_R, GLASS),
    (NECK_GRIP_Z1, 0.170, 0.0135, GLASS),
    (0.170, SEAT - BEAD_H, NECK_R, GLASS),
    (SEAT - BEAD_H, SEAT, BEAD_R, GLASS),
]

BOTTLE = part(
    "bottle", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001), mass=BOTTLE_MASS, color=GLASS, friction=1.0,
    boxes=[((0.0, 0.0, 0.055), (0.03, 0.03, 0.08), GLASS)],
    cylinders=[_cyl(z0, z1, r, c) for z0, z1, r, c in BOTTLE_PROFILE])


def _skirt():
    r = CAP_R - SKIRT_T / 2
    w = 2 * r * math.tan(math.pi / FLUTES) + 0.0004
    out = []
    for i in range(FLUTES):
        a = 2 * math.pi * i / FLUTES  # a flat face toward the jaw (+x), not a corner
        q = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))  # (w, x, y, z), partkit's box rotation
        out.append(((r * math.cos(a), r * math.sin(a), -SKIRT_H / 2), (SKIRT_T, w, SKIRT_H), CAP_RED, q))
    return out


CAP = part(
    "cap", pos=(BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001 + SEAT), mass=CAP_MASS, color=CAP_RED, friction=0.8,
    boxes=_skirt(), cylinders=[_cyl(0.0, TOP_T, CAP_R, CAP_RED)])

# the crimp: the cap's origin held at the middle of the finish's top, no turning about the bottle's axis, tilting
# against CRIMP_K (authored by partkit when the cap is spawned; _follow switches it off when it lets go)
CRIMP = tilt_joint("crimp", BOTTLE.name, CAP.name, anchor0=(0.0, 0.0, SEAT), anchor1=(0.0, 0.0, 0.0),
                   axis=(0.0, 0.0, 1.0), stiffness=CRIMP_K, damping=CRIMP_DAMPING)

OBJECTS = [
    Box("post", pos=(WALL_X + POST_T / 2, JAW_Y, (TOP + POST_TOP) / 2), size=(POST_T, POST_W, POST_TOP - TOP),
        color=POST),
    OPENER,
    Cylinder("coaster", pos=(COASTER[0], COASTER[1], TOP + COASTER_H / 2), radius=COASTER_R, height=COASTER_H,
             color=CORK),
    BOTTLE,
    CAP,
]


# -- the trial record, followed after every control step -----------------------------------------------------------------

def _fresh() -> dict:
    return {"engaged": False, "released": False, "popped": False, "tipped": False, "steps": 0,
            "release_step": None, "last_caught": None, "moment": 0.0, "max_moment": 0.0, "max_tilt": 0.0,
            "tcps": [], "gap": -1.0, "gap_part": None, "fk_error": 0.0}


_S = _fresh()
_DEBUG = bool(os.environ.get("BOTTLE_DEBUG"))


def _tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def crimp_tilt(bottle, cap) -> float:
    """How far (rad) the cap is tilted on the finish: the angle between the cap's axis and the bottle's."""
    c = float(np.clip(bottle.rotation()[:, 2] @ cap.rotation()[:, 2], -1.0, 1.0))
    return math.acos(c)


def caught(cap) -> bool:
    """The prying lip is under the edge of the cap's skirt. In the cap's own frame (it tips as it is pried), the lip's
    edge, where it passes the cap, is 12.8 to 14.2 mm out from the cap's axis (under the skirt's wall, outside the bead)
    and 0 to 2.5 mm beyond the skirt's edge (-0.5 mm allows for contact); and the cap's axis is within 6 mm of the
    jaw's centre line sideways."""
    c = np.asarray(cap.pos, dtype=np.float64)
    y = float(np.clip(c[1] - JAW_Y, -0.009, 0.009))
    p = cap.to_local(jaw_to_world((LIP_TIP_J, y, -SKIRT_H - TOP_T - JAW_GAP)))
    reach = math.hypot(float(p[0]), float(p[1]))
    under = -SKIRT_H - float(p[2])
    return (CATCH_IN[0] < reach < CATCH_IN[1] and abs(float(c[1]) - JAW_Y) < CATCH_SIDE
            and CATCH_GAP[0] < under < CATCH_GAP[1])


def bottle_gap(points, bottle) -> np.ndarray:
    """Signed distance (m) from world points to the bottle's glass, the union of BOTTLE_PROFILE's cylinders in the
    bottle's frame: positive outside, negative inside."""
    local = (np.asarray(points, dtype=np.float64) - np.asarray(bottle.pos, dtype=np.float64)) @ bottle.rotation()
    r, z = np.hypot(local[:, 0], local[:, 1]), local[:, 2]
    out = np.full(len(local), np.inf)
    for z0, z1, rad, _ in BOTTLE_PROFILE:
        dr, dz = r - rad, np.maximum(z0 - z, z - z1)
        out = np.minimum(out, np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0)) + np.minimum(np.maximum(dr, dz), 0.0))
    return out


_ARM_SAMPLES: dict = {}


def arm_samples() -> dict[str, tuple[np.ndarray, float, np.ndarray, float]]:
    """(points, spacing, centre, radius) per moving link: points on the surface of the link's collision hulls in the
    link's frame, sampled on a grid of that spacing, and a ball holding them all. The hulls are the YAM URDF's CoACD
    convex hulls, which the simulator's arm colliders are converted from as-is. Points inside another hull of the same
    link are dropped: they can never be the link's nearest point to the bottle."""
    if _ARM_SAMPLES:
        return _ARM_SAMPLES
    import xml.etree.ElementTree as ET

    from yam_arena import ASSET_DIR
    from yam_arena.analysis import read_stl
    from yam_arena.urdf import rpy_matrix

    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ARM_LINKS:
            continue
        spacing, hulls = ARM_LINKS[name], []
        for col in link.findall("collision"):
            origin = col.find("origin")
            rot, off = np.eye(3), np.zeros(3)
            if origin is not None:
                rot = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                off = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
            hulls.append(read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ rot.T + off)
        planes = []
        for tris in hulls:  # each hull's outward face planes (n . x <= d inside)
            n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
            keep = np.linalg.norm(n, axis=1) > 1e-12
            n = n[keep] / np.linalg.norm(n[keep], axis=1, keepdims=True)
            d = np.einsum("ij,ij->i", n, tris[keep, 0])
            flip = n @ tris.reshape(-1, 3).mean(axis=0) - d > 0
            n[flip], d[flip] = -n[flip], -d[flip]
            planes.append((n, d))
        pts = []
        for h, tris in enumerate(hulls):
            edge = np.max(np.linalg.norm(tris - np.roll(tris, 1, axis=1), axis=2), axis=1)
            steps = np.maximum(1, np.ceil(edge / spacing).astype(int))
            for k in np.unique(steps):  # a barycentric grid of k x k per triangle
                t = tris[steps == k]
                ij = np.array([(i, j) for i in range(k + 1) for j in range(k + 1 - i)], dtype=np.float64) / k
                p = t[:, None, 0] + ij[None, :, :1] * (t[:, None, 1] - t[:, None, 0]) \
                    + ij[None, :, 1:] * (t[:, None, 2] - t[:, None, 0])
                p = p.reshape(-1, 3)
                inside = np.zeros(len(p), dtype=bool)
                for g, (n, d) in enumerate(planes):
                    if g != h:
                        inside |= (p @ n.T - d).max(axis=1) < -1e-4
                pts.append(p[~inside])
        p = np.concatenate(pts)
        p = p[np.unique(np.round(p / 1e-6).astype(np.int64), axis=0, return_index=True)[1]]  # shared corners/edges
        centre = p.mean(axis=0)
        _ARM_SAMPLES[name] = (p, spacing, centre, float(np.linalg.norm(p - centre, axis=1).max()))
    missing = set(ARM_LINKS) - set(_ARM_SAMPLES)
    if missing:
        raise KeyError(f"no collision hulls for {sorted(missing)} in {YAM_URDF}")
    return _ARM_SAMPLES


def arms_clearance(link_poses: dict[str, dict[str, tuple]], bottle) -> tuple[float, str]:
    """(gap, part): how far the nearest part of either arm is from the bottle's glass (m), less each link's sampling
    allowance, and which arm and link that is. ``link_poses`` maps arm -> link -> (position, 3x3 rotation matrix) in
    the env frame."""
    samples = arm_samples()
    middle = np.asarray(bottle.to_world((0.0, 0.0, SEAT / 2)), dtype=np.float64)
    best, part_name = np.inf, "none"
    for arm, poses in link_poses.items():
        for name, (pts, spacing, centre, radius) in samples.items():
            pos = np.asarray(poses[name][0], dtype=np.float64)
            rot = np.asarray(poses[name][1], dtype=np.float64)
            # the bottle fits in a 10 cm ball round its mid-height: skip a link whose own ball is well clear of it
            if np.linalg.norm(rot @ centre + pos - middle) - radius > 0.10 + LET_GO + 0.02:
                continue
            gap = float(bottle_gap(pts @ rot.T + pos, bottle).min()) - 0.6 * spacing
            if gap < best:
                best, part_name = gap, f"{arm}/{name}"
    return best, part_name


def arm_link_poses(joints: dict[str, float], base_pos) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Link poses (position, 3x3 rotation, env frame) of one YAM arm with these joint angles (joint1..joint8), its base
    link at ``base_pos``: the URDF's forward kinematics, which the simulator's links follow (yam_arena checks them
    against each other)."""
    fk = forward_kinematics(YAM_URDF, {k: float(v) for k, v in joints.items()})
    base = np.asarray(base_pos, dtype=np.float64)
    return {n: (BASE_ROT @ fk[n][:3, 3] + base, BASE_ROT @ fk[n][:3, :3]) for n in ARM_LINKS}


def _link_poses(environment) -> tuple[dict[str, dict[str, tuple]], float]:
    """Both arms' link poses (position, 3x3 rotation) relative to the env origin, for the links in ARM_LINKS, and the
    largest gap (m) between a link's position from the kinematics and the simulator's. The poses come from each arm's
    measured joint angles through the URDF's forward kinematics, from its base link's position: no simulator quaternion
    is read, so they can't depend on a quaternion order. The simulator's link positions (which have no order) check
    them."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out, worst = {}, 0.0
    for key in ARMS:
        art = base.scene[key]
        names = list(art.body_names)
        sim_pos = _np(art.data.body_link_pose_w)[0][:, :3] - origin  # positions only
        joints = dict(zip(list(art.joint_names), _np(art.data.joint_pos)[0].tolist(), strict=True))
        links = arm_link_poses(joints, sim_pos[names.index("base")])
        worst = max(worst, max(float(np.linalg.norm(links[n][0] - sim_pos[names.index(n)])) for n in ARM_LINKS))
        out[key] = links
    return out, worst


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the pads, 13 mm above the pad tips), for the debug trace."""
    base = environment.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def _seat_cap(environment, bottle) -> None:
    x, y, z, w = (float(v) for v in bottle.quat_xyzw)
    write_pose(environment, CAP.name, tuple(float(v) for v in bottle.to_world((0.0, 0.0, SEAT))), (x, y, z, w),
               lin_vel=tuple(float(v) for v in bottle.lin_vel), ang_vel=tuple(float(v) for v in bottle.ang_vel))


@on_reset
def _reset(environment=None):
    was_released = _S.get("released", False)
    _S.clear()
    _S.update(_fresh())
    arm_samples()  # sampled once, here (a few seconds), not in the middle of a trial
    if environment is not None:
        if was_released:  # the crimp let go last episode: put it back
            set_joint_enabled(CRIMP.name, True)
        bottle = _part_state(environment, _SPECS[BOTTLE.name])
        if _tilt_deg(bottle) > 90.0:
            raise RuntimeError("the standing bottle reads upside down: the simulator's pose quaternions are not in "
                               "the (x, y, z, w) order of Isaac Lab 3.0 that this task reads")
        _seat_cap(environment, bottle)


@on_step
def _follow(environment) -> None:
    """The crimp and the let-go: while the crimp holds, the moment on the cap is its tilt on the finish times CRIMP_K;
    when the tilt reaches THETA_POP (the moment T_OPEN) the crimp is switched off for good. That opened the bottle on the
    opener if the prying lip was under the skirt then (or within CAUGHT_RECENT steps). Once it is open, every step
    measures how close either arm is to the bottle."""
    _S["steps"] += 1
    bottle = _part_state(environment, _SPECS[BOTTLE.name])
    cap = _part_state(environment, _SPECS[CAP.name])
    tilt = _tilt_deg(bottle)
    _S["max_tilt"] = max(_S["max_tilt"], tilt)
    if tilt > TIPPED_DEG or bottle.pos[2] < TOP - 0.05:
        _S["tipped"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    if not _S["released"]:
        if caught(cap):
            _S["engaged"] = True
            _S["last_caught"] = _S["steps"]
        bend = crimp_tilt(bottle, cap)
        _S["moment"] = CRIMP_K * bend
        _S["max_moment"] = max(_S["max_moment"], _S["moment"])
        if bend >= THETA_POP:
            _S["released"], _S["release_step"] = True, _S["steps"]
            _S["popped"] = _S["last_caught"] is not None and _S["steps"] - _S["last_caught"] <= CAUGHT_RECENT
            set_joint_enabled(CRIMP.name, False)
            print(f"[bottle] crimp let go at step {_S['steps']}: moment {_S['moment']:.2f} N m, tilt on the finish "
                  f"{math.degrees(bend):.1f} deg, bottle tilt {tilt:.1f} deg, under the lip: {_S['popped']}",
                  flush=True)
    if _S["popped"]:  # only the open bottle's let-go is scored
        poses, _S["fk_error"] = _link_poses(environment)
        if _S["fk_error"] > FK_TOL:  # never let go on poses we can't trust
            _S["gap"], _S["gap_part"] = -np.inf, "fk-mismatch"
            print(f"[bottle] WARNING arm kinematics {(_S['fk_error'] * 1000):.1f} mm off the simulator's links",
                  flush=True)
        else:
            _S["gap"], _S["gap_part"] = arms_clearance(poses, bottle)
    if _DEBUG and _S["engaged"] and _S["release_step"] in (None, _S["steps"]):  # every step while it is pried
        print(f"[bottle] crimp step {_S['steps']} moment {_S['moment']:.3f} N m caught {caught(cap)}", flush=True)
    if _DEBUG and _S["steps"] % 5 == 0:
        print(f"[bottle] step {_S['steps']} bottle {np.round(bottle.pos, 4)} tilt {tilt:.1f} "
              f"cap {np.round(cap.pos, 4)} caught {caught(cap)} moment {_S['moment']:.2f}Nm "
              f"(max {_S['max_moment']:.2f}) engaged {_S['engaged']} released {_S['released']} popped {_S['popped']} "
              f"gap {_S['gap'] * 1000:.1f}mm {_S['gap_part']} fk {_S['fk_error'] * 1000:.2f}mm "
              f"score {score({BOTTLE.name: bottle}):.2f} tcps {[np.round(t, 3).tolist() for t in _S['tcps']]}",
              flush=True)


def on_coaster(bottle) -> bool:
    dx, dy = float(bottle.pos[0] - COASTER[0]), float(bottle.pos[1] - COASTER[1])
    base_z = float(bottle.pos[2])
    return (math.hypot(dx, dy) < ON_COASTER and TOP + COASTER_H - 0.003 < base_z < TOP + COASTER_H + 0.006
            and _tilt_deg(bottle) < UPRIGHT_DEG)


def released() -> bool:
    """Neither arm holds or touches the bottle: no part of either (fingers, gripper, wrist or arm) within LET_GO of any
    part of its glass, body included."""
    return _S["gap"] >= LET_GO


def score(objects) -> float:
    bottle = objects[BOTTLE.name]
    if _S["tipped"]:
        return 0.0
    if _S["released"] and not _S["popped"]:
        return 0.0  # the crimp was pried open somewhere other than under the opener's lip
    if not _S["popped"]:
        return 0.25 if _S["engaged"] else 0.0
    if not on_coaster(bottle):
        return 0.5
    if not (released() and bottle.resting()):
        return 0.75
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo (right arm) -----------------------------------------------------------------------------------
# The right hand holds the bottle by its body and uses it as the lever, as a person does on a wall opener. Its fingers
# close across the body (along y), its approach tipped GRIP_PITCH from the bottle's axis, so with the bottle tipped
# TILT_IN for the mouth the hand points straight down. It pushes the cap up into the mouth beside the lip until the
# cap's top meets the pad, moves it toward the lip until the skirt meets the mouth's edge (the lip is then under the
# skirt's edge), then presses the bottle down toward upright, pivoting on the pad, DEPRESS_DEG in all: the crimp holds the
# bottle back about 9.5 degrees in, so the arm's give presses about 12 N on the body 12 cm from the pad, which levers the
# crimp open. (A press that moved the whole bottle on past that would drive the open bottle into the post.)

GRIP_PITCH = math.radians(53.0)  # the approach, from the bottle's axis toward its +x (equal to TILT_IN: straight down)
GRIP_UP = 0.065  # TCP above the bottle's base: the middle of the body, in the label band
COM_UP = 0.066  # the full bottle's centre of mass above its base (from its shapes)
PAYLOAD = BOTTLE_MASS + CAP_MASS
NECK_GRIP_UP = 0.154  # neck_hold: a straight-down grip on the lower neck, 3.7 cm under the cap's top
CARRY_UP = 0.035  # the bottle carried this far above the table
INTO_BESIDE = 0.0025  # into the mouth with the cap this far off the lip's side (J x), its edge clear of the lip
INTO_PAST = 0.0015  # aim the cap's top this far past the pad: the arm presses it on the pad
HOOK_PAST = 0.0005  # aim the skirt this far past the mouth's edge: the arm presses it there, the lip under its edge
PIVOT_J = (NOSE_HALF, 0.0, 0.0)  # the bottle pivots on the pad's nose, at its edge nearest the wall
DEPRESS_DEG = float(os.environ.get("BOTTLE_DEPRESS_DEG", "20"))
PUSH = float(os.environ.get("BOTTLE_PUSH", "0.0"))  # m: an extra press along the turn at the end (none by default)
DEPRESS_SHORT_DEG = 4.5  # pry_short: the skirt's edge just onto the lip, and no further
OUT = 0.030  # after it lets go: back out of the mouth along the bottle's axis this far
# The probe run's measured misses (m, x y z, actual - wanted) left over after the static sag model: the cap's in front of
# the mouth, the bottle base's over the coaster. The run book (calibrate.py) sets them; they default to none.
FIX_OPENER = np.array([float(v) for v in os.environ.get("BOTTLE_FIX_OPENER", "0,0,0").split(",")])
FIX_COASTER = np.array([float(v) for v in os.environ.get("BOTTLE_FIX_COASTER", "0,0,0").split(",")])
RIGHT_BASE = np.array(station.BASES["right"])
JOINT_K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
PARK = (0.18, -0.34, TOP + 0.15)  # the right hand, out of the way


_ry = _rot_y
_DOWN = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP))


def grip_R(tip: float = 0.0) -> np.ndarray:
    """The TCP rotation holding the bottle tipped ``tip`` (rad, top toward the wall, about +y): the body grip, its
    approach GRIP_PITCH from the bottle's axis toward the bottle's +x, turned with the bottle."""
    return _ry(tip) @ _ry(-GRIP_PITCH) @ _DOWN


class Grip(float):
    """A Waypoint ``yaw`` that carries the whole TCP rotation ``R`` (world): yam_arena's demo knows only straight-down
    grips turned about the vertical, so its grip_rotation is wrapped below to take one of these."""

    R: np.ndarray

    def __new__(cls, R):
        obj = super().__new__(cls, 0.0)
        obj.R = np.asarray(R, dtype=np.float64)
        return obj


if not hasattr(_demo, "_plain_grip_rotation"):
    _demo._plain_grip_rotation = _demo.grip_rotation


def _grip_rotation(yaw=0.0) -> np.ndarray:
    R = getattr(yaw, "R", None)
    return R if R is not None else _demo._plain_grip_rotation(float(yaw))


_demo.grip_rotation = _grip_rotation
_COM_TCP = grip_R(0.0).T @ np.array([0.0, 0.0, COM_UP - GRIP_UP])  # the bottle's centre of mass in the TCP frame


def sagged_joints(q_cmd) -> np.ndarray:
    """Where the right arm's joints settle holding the full bottle by its body: the joints are springs (80 N m/rad at
    the shoulder and elbow, 10 at the wrist), with no compensation for a payload."""
    force = np.array([0.0, 0.0, -PAYLOAD * 9.81])

    def com_world(q):
        p, R = ik.tcp_pose(q, RIGHT_BASE)
        return p + R @ _COM_TCP

    q = np.array(q_cmd, dtype=np.float64)
    for _ in range(20):
        c0 = com_world(q)
        J = np.zeros((3, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += 1e-6
            J[:, i] = (com_world(dq) - c0) / 1e-6
        q = np.asarray(q_cmd, dtype=np.float64) + (J.T @ force) / JOINT_K
    return q


def compensate(pos, R) -> tuple[np.ndarray, np.ndarray]:
    """The pose to command so that the loaded arm sags onto (pos, R): aim the other way, position and whole rotation."""
    want_p, want_R = np.asarray(pos, dtype=np.float64), np.asarray(R, dtype=np.float64)
    aim_p, aim_R, q = want_p.copy(), want_R.copy(), None
    for _ in range(8):
        q, _, _ = ik.solve(aim_p, ik.matrix_to_quat_xyzw(aim_R), q_init=q, base_pos=RIGHT_BASE, iters=200)
        p1, R1 = ik.tcp_pose(sagged_joints(q), RIGHT_BASE)
        aim_p = aim_p + (want_p - p1)
        aim_R = (want_R @ R1.T) @ aim_R
    return aim_p, aim_R


def _axis(tip: float) -> np.ndarray:
    return _ry(tip) @ np.array([0.0, 0.0, 1.0])


def _tcp_for(base, tip: float = 0.0) -> np.ndarray:
    """The TCP for the bottle's base at ``base`` (world), the bottle tipped ``tip``: GRIP_UP up its axis."""
    return np.asarray(base, dtype=np.float64) + GRIP_UP * _axis(tip)


def _base_for_top(top, tip: float) -> np.ndarray:
    """The bottle's base with the middle of its cap's top at ``top`` (world), the bottle tipped ``tip``."""
    return np.asarray(top, dtype=np.float64) - (SEAT + TOP_T) * _axis(tip)


def _tipped_about(base, pivot, turn: float) -> np.ndarray:
    """The bottle's base after the bottle is turned ``turn`` about +y through the point ``pivot`` (world)."""
    p = np.asarray(pivot, dtype=np.float64)
    return p + _ry(turn) @ (np.asarray(base, dtype=np.float64) - p)


def _w(duration, pos, finger, label, R, fix=None) -> Waypoint:
    """A waypoint with the bottle in the hand: aimed so the loaded arm sags onto it, less a measured miss."""
    p, aim = compensate(pos, R)
    if fix is not None:
        p = p - fix
    return Waypoint(duration, tuple(float(v) for v in p), finger, label, Grip(aim))


def _free(duration, pos, finger, label, R) -> Waypoint:
    return Waypoint(duration, tuple(float(v) for v in pos), finger, label, Grip(R))


def _pick() -> list[Waypoint]:
    R0 = grip_R(0.0)
    tcp = _tcp_for((BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001))
    back = tcp - 0.07 * R0[:, 2]  # 7 cm back along the approach: the open fingers clear the body
    return [
        _free(2.0, back, FINGER_OPEN, "behind the bottle", R0),
        _free(1.5, tcp, FINGER_OPEN, "fingers round the body", R0),
        _free(0.8, tcp, FINGER_CLOSED, "grip the body", R0),
    ]


def _mouth_tops() -> dict[str, np.ndarray]:
    """Where the middle of the cap's top goes (world), the bottle tipped TILT_IN, at each step at the opener."""
    return {"front": jaw_to_world((INTO_BESIDE, 0.0, -0.015)),  # in front of the mouth, 15 mm short of the pad
            "in": jaw_to_world((INTO_BESIDE, 0.0, INTO_PAST)),  # up into the mouth, on the pad
            "hooked": jaw_to_world((-HOOK_PAST, 0.0, INTO_PAST))}  # over to the mouth's edge: the lip under the skirt


def probe_windows() -> dict[str, tuple[float, float, np.ndarray]]:
    """For the probe run: (start s, end s, wanted position) of the two still holds, the cap's origin in front of the
    mouth and the bottle's base over the coaster; calibrate.py compares the trace with them."""
    front = _mouth_tops()["front"] - TOP_T * _axis(TILT_IN)  # the cap's origin, under its top
    t_front = 2.0 + 1.5 + 0.8 + 1.2 + 2.0 + 1.8  # the pick, the lift, the tip and the carry
    t_coast = t_front + 3.0 + 1.2 + 2.0
    return {"opener": (t_front + 1.5, t_front + 3.0, front),
            "coaster": (t_coast + 1.5, t_coast + 3.0, np.array([COASTER[0], COASTER[1], TOP + COASTER_H + 0.03]))}


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        return {"right": [Waypoint(3.0, (0.25, -0.30, TOP + 0.23), FINGER_OPEN, "wait")]}
    if variant == "pull_cap":
        return {"right": _pull_cap()}
    R0, Rt = grip_R(0.0), grip_R(TILT_IN)
    tops = _mouth_tops()
    start = np.array([BOTTLE_AT[0], BOTTLE_AT[1], TOP + 0.001])
    coaster = np.array([COASTER[0], COASTER[1], TOP + COASTER_H])
    front = _base_for_top(tops["front"], TILT_IN)
    tipped_away = front - np.array([0.06, 0.0, 0.0])  # tipped over 6 cm short of the opener
    right = _pick() + [
        _w(1.2, _tcp_for(start + np.array([0.0, 0.0, CARRY_UP])), FINGER_CLOSED, "lift", R0),
        _w(2.0, _tcp_for(tipped_away, TILT_IN), FINGER_CLOSED, "tip it over, top toward the post", Rt),
    ]
    if variant == "probe":  # still holds in front of the mouth and over the coaster, for calibrate.py
        return {"right": right + [
            _w(1.8, _tcp_for(front, TILT_IN), FINGER_CLOSED, "in front of the mouth", Rt),
            _w(3.0, _tcp_for(front, TILT_IN), FINGER_CLOSED, "hold still", Rt),
            _w(1.2, _tcp_for(front - np.array([0.05, 0.0, 0.0]), TILT_IN), FINGER_CLOSED, "away from the post", Rt),
            _w(2.0, _tcp_for(coaster + np.array([0.0, 0.0, 0.03])), FINGER_CLOSED, "upright, over the coaster", R0),
            _w(3.0, _tcp_for(coaster + np.array([0.0, 0.0, 0.03])), FINGER_CLOSED, "hold still", R0),
        ]}
    if variant != "skip_opener":
        right += [
            _w(1.8, _tcp_for(front, TILT_IN), FINGER_CLOSED, "in front of the mouth", Rt, FIX_OPENER),
            _w(0.8, _tcp_for(front, TILT_IN), FINGER_CLOSED, "steady", Rt, FIX_OPENER),
            _w(2.0, _tcp_for(_base_for_top(tops["in"], TILT_IN), TILT_IN), FINGER_CLOSED,
               "cap up into the mouth, its top onto the pad", Rt, FIX_OPENER),
        ]
        hooked = _base_for_top(tops["in"] if variant == "no_hook" else tops["hooked"], TILT_IN)
        if variant != "no_hook":  # no_hook: the lip is never put under the skirt
            right += [_w(1.5, _tcp_for(hooked, TILT_IN), FINGER_CLOSED, "over onto the lip", Rt, FIX_OPENER)]
        pivot = jaw_to_world(PIVOT_J)
        first = -math.radians(DEPRESS_SHORT_DEG if variant == "pry_short" else min(12.0, DEPRESS_DEG))
        pressed = _tipped_about(hooked, pivot, first)
        right += [_w(2.5, _tcp_for(pressed, TILT_IN + first), FINGER_CLOSED,
                     "press it down toward upright, on the pad", grip_R(TILT_IN + first), FIX_OPENER)]
        turn = -math.radians(DEPRESS_DEG)
        tip = TILT_IN + turn
        if variant == "pry_short":  # stops there: the skirt's edge just onto the lip, short of the crimp's strength
            return {"right": right + [_w(3.0, _tcp_for(pressed, TILT_IN + first), FINGER_CLOSED, "hold",
                                         grip_R(TILT_IN + first), FIX_OPENER)]}
        pressed = _tipped_about(hooked, pivot, turn)
        g = _tcp_for(pressed, tip)
        along = np.cross([0.0, -1.0, 0.0], g - pivot)  # the way the grip moves as the bottle turns toward upright
        pushed = pressed + PUSH * along / np.linalg.norm(along)
        out = pushed - OUT * _axis(tip)
        right += [
            _w(2.0, _tcp_for(pushed, tip), FINGER_CLOSED, "press on: the crimp lets go", grip_R(tip), FIX_OPENER),
            _w(0.6, _tcp_for(pushed, tip), FINGER_CLOSED, "hold", grip_R(tip), FIX_OPENER),
            _w(1.2, _tcp_for(out, tip), FINGER_CLOSED, "back out of the mouth", grip_R(tip), FIX_OPENER),
            _w(1.2, _tcp_for(out - np.array([0.05, 0.0, 0.0]), tip), FINGER_CLOSED, "away from the post",
               grip_R(tip), FIX_OPENER),
        ]
        if variant == "no_place":
            return {"right": right + [_w(3.0, _tcp_for(out - np.array([0.05, 0.0, 0.0]), tip), FINGER_CLOSED,
                                         "hold it", grip_R(tip), FIX_OPENER)]}
    above = coaster + np.array([0.0, 0.0, 0.03])
    down = coaster + np.array([0.0, 0.0, 0.002])
    set_down = _w(1.5, _tcp_for(down), FINGER_CLOSED, "set down", R0, FIX_COASTER)
    right += [_w(2.0, _tcp_for(above), FINGER_CLOSED, "upright, over the coaster", R0, FIX_COASTER), set_down]
    if variant == "neck_hold":  # the hand slides from the body up to the lower neck, never 1 cm clear of the glass
        return {"right": right + _slide_to_neck(set_down)}
    if variant == "no_release":  # keeps holding it by the body
        return {"right": right + [Waypoint(3.0, set_down.pos, FINGER_CLOSED, "keep holding the body", set_down.yaw)]}
    # the fingers open where the loaded arm was aimed, so the hand doesn't rise while they still hold the bottle
    tcp = np.asarray(set_down.pos)
    right += [Waypoint(0.8, set_down.pos, FINGER_OPEN, "let go", set_down.yaw)]
    if variant == "touch_body":  # one open finger's pad pressed 2 mm onto the body
        return {"right": right + [
            _free(1.2, tcp + np.array([0.0, 0.020, 0.0]), FINGER_OPEN, "pad onto the body", R0),
            _free(3.0, tcp + np.array([0.0, 0.020, 0.0]), FINGER_OPEN, "keep touching it", R0),
        ]}
    back = tcp - 0.08 * R0[:, 2]
    right += [_free(1.2, back, FINGER_OPEN, "back off along the approach", R0)]
    plan = {"right": right + [_free(1.8, PARK, FINGER_OPEN, "clear", R0)]}
    if variant == "full":  # the left wrist camera watches the mouth while the cap comes off
        plan["left"] = _left_watch([w.duration for w in plan["right"]], [w.label for w in plan["right"]])
    return plan


# neck_hold: the openings of the loosened fingers (m, the pads' distance from the TCP) at the TCP's heights above the
# bottle's base on the way up: about 4 mm off the glass (body r 29, shoulder r 20, lower neck r 16 mm). The hand closes
# with its TCP 15.3 cm up, where the pads only meet the straight part of the lower neck (14 to 16.5 cm), above the
# shoulder's taper, which closing pads would wedge the bottle up.
NECK_SLIDE = [(0.100, 0.033, 0.9, "slide up the body"), (0.114, 0.033, 0.5, "to the shoulder"),
              (0.128, 0.026, 0.6, "over the shoulder"), (0.153, 0.021, 0.7, "onto the lower neck")]
NECK_SETTLE = 0.008  # then the closed hand presses down this far: the base stays flat on the coaster


def _slide_to_neck(set_down: Waypoint) -> list[Waypoint]:
    """From the body grip on the coaster to a grip on the lower neck without letting go. The hand first stops pushing:
    still closed, its aim moves from the loaded arm's (set_down) to where the bottle stands, so it doesn't jump when the
    fingers loosen. The fingers loosen to 4 mm off the glass, the hand slides straight up the bottle with the fingers
    following its profile, closes on the lower neck and presses the bottle down onto the coaster."""
    R0 = grip_R(0.0)

    def at(h):
        return (COASTER[0], COASTER[1], TOP + COASTER_H + h)

    out = [_free(0.8, at(GRIP_UP), FINGER_CLOSED, "rest it on the coaster", R0),
           _free(0.5, at(GRIP_UP), -NECK_SLIDE[0][1], "loosen the fingers", R0)]
    out += [_free(t, at(h), -w, label, R0) for h, w, t, label in NECK_SLIDE]
    h = NECK_SLIDE[-1][0]
    return out + [_free(0.6, at(h), FINGER_CLOSED, "grip the neck", R0),
                  _free(0.8, at(h - NECK_SETTLE), FINGER_CLOSED, "press it down onto the coaster", R0),
                  _free(3.0, at(h - NECK_SETTLE), FINGER_CLOSED, "keep holding the neck", R0)]


# The full demo's left hand only carries its wrist camera to watch the cap come off (from the top camera the right arm
# hides it). The camera looks into the mouth from the post's left and 10 degrees below the mouth's plane (the jaw frame's
# x-y plane), 27 cm from the lip, the image upright. From there it sees, through the gap under the mouth's left side,
# the lip's edge under the skirt and the skirt's edge by it, and the cap as it falls. Checked offline against the
# collision hulls of the opener, the post, the cap, the bottle and the right arm over the pry: the lip's edge projects at
# about (317, 177) of 640 x 360, unoccluded, and the left arm stays 14 cm or more from the opener and the post and 9 cm
# or more from the bottle and the right arm.
LEFT_BASE = np.array(station.BASES["left"])
WATCH_FROM = (0.0, math.cos(math.radians(10.0)), -math.sin(math.radians(10.0)))  # jaw frame: from the left, a bit below
WATCH_D = 0.27  # the camera this far from the lip
WATCH_ROLL = 90.0  # degrees about the view axis: the image upright


def _watch_pose() -> tuple[np.ndarray, np.ndarray]:
    """The left TCP pose (position, rotation) that puts the left wrist camera on the lip, as above."""
    from yam_arena.constants import TCP_OFFSET

    aim = np.mean([jaw_to_world((-0.0130, y, -0.0075)) for y in (-0.006, 0.0, 0.006)]
                  + [jaw_to_world((-0.0149 * math.cos(a), 0.0149 * math.sin(a), -0.0059))
                     for a in np.radians((-35.0, 0.0, 35.0))], axis=0)
    u = R_J @ np.asarray(WATCH_FROM, dtype=np.float64)
    u = u / np.linalg.norm(u)
    cam_at = aim + WATCH_D * u
    x = np.cross([0.0, 0.0, 1.0], u)
    x = x / np.linalg.norm(x)
    c, s_ = math.cos(math.radians(WATCH_ROLL)), math.sin(math.radians(WATCH_ROLL))
    r_cam = np.stack([x, np.cross(u, x), u], 1) @ np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    mount = station.wrist_camera("left")  # the camera in the gripper link's frame (OpenGL: it looks along its -z)
    r_grip = r_cam @ quat_xyzw_to_matrix(np.asarray(mount.rot_xyzw)).T
    grip_at = cam_at - r_grip @ np.asarray(mount.pos)
    return grip_at + r_grip @ np.asarray(TCP_OFFSET), r_grip


def _left_watch(durations, labels) -> list[Waypoint]:
    """The left arm, from its ready pose: there before the cap goes into the mouth, still while it is pried off and
    drops, and back before the bottle is carried to the coaster."""
    t = np.cumsum(durations)
    t_in = float(t[labels.index("steady")])  # the cap starts up into the mouth after this
    t_out = float(t[labels.index("back out of the mouth")])
    p0, r0 = ik.tcp_pose(np.array(READY, dtype=np.float64), LEFT_BASE)
    p, r = _watch_pose()
    go = 3.0
    return [Waypoint(t_in - go - 0.6, tuple(float(v) for v in p0), FINGER_CLOSED, "wait", Grip(r0)),
            Waypoint(go, tuple(float(v) for v in p), FINGER_CLOSED, "to watch the mouth", Grip(r)),
            Waypoint(t_out - t_in + 0.6, tuple(float(v) for v in p), FINGER_CLOSED, "watch the cap come off", Grip(r)),
            Waypoint(1.8, tuple(float(v) for v in p0), FINGER_CLOSED, "back", Grip(r0))]


def _pull_cap() -> list[Waypoint]:
    """Pinch the cap and pull it straight up: the crimp holds, so the bottle comes up with it."""
    bx, by = BOTTLE_AT
    z_cap = TOP + 0.001 + SEAT + 0.004
    hover = TOP + 0.23
    return [
        Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the cap"),
        Waypoint(1.5, (bx, by, z_cap), FINGER_OPEN, "fingers round the cap"),
        Waypoint(0.8, (bx, by, z_cap), FINGER_CLOSED, "pinch it"),
        Waypoint(2.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "pull it straight up"),
        Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_CLOSED, "hold"),
        Waypoint(1.0, (bx, by, z_cap + 0.05), FINGER_OPEN, "let go"),
        Waypoint(1.8, (0.12, -0.42, TOP + 0.24), FINGER_OPEN, "clear"),
    ]

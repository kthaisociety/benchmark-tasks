"""On the station table two aluminium forearm crutches lie side by side on a fixed wooden stand, left to right, their
rubber tips to the right and their cuffs to the left. Each lies with its handgrip hanging straight down between the
stand's two cradles, so the back of the crutch, with the leg's adjustment holes and spring button, faces up.

A forearm crutch's length is set by its leg: a thinner tube that slides inside the main tube and is held by a spring
button on it, which stands out through one of a row of nine holes in the main tube. Pressed flush, the button lets
the leg slide; let go over a hole, it springs out through it and locks the leg. Both crutches are to be set to the
hole ringed in red (the fifth from the bottom): the near crutch starts in the second hole (three holes too long), the
far one in the eighth (three holes too short).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Set both crutches to the hole ringed in red: each leg's spring button must stand fully out through "
               "that hole. Then let go of the crutches.")

ALU = (0.72, 0.74, 0.77)
LEG_C = (0.55, 0.57, 0.60)
BLACK = (0.08, 0.08, 0.09)
GRIP_C = (0.16, 0.17, 0.19)
CUFF_C = (0.22, 0.24, 0.27)
BUTTON_C = (0.80, 0.80, 0.82)
RED = (0.80, 0.08, 0.08)
WOOD = (0.62, 0.47, 0.30)

# -- the crutch, in its own frame: origin on the main tube's axis at its lower end (the collar's end); +y up the crutch
# toward the cuff; +z is the crutch's back (up on the stand), -z its front (the handgrip points that way, down) -------
R_OUT = 0.0111  # main tube: 22.2 mm (7/8 in) across ...
WALL = 0.0012  # ... with a 1.2 mm wall
R_INNER = R_OUT - WALL  # 9.9 mm: the main tube's inside
LEG_R = 0.0095  # the leg: a 19 mm (3/4 in) tube, 0.4 mm smaller all round than the main tube's inside
N_HOLES, PITCH = 9, 0.0254  # nine adjustment holes, 1 in apart ...
HOLE1_Y = 0.040  # ... the lowest 4 cm above the main tube's end
HOLE_W = 0.0070  # each hole 7 mm across
TARGET = 5  # the hole ringed in red, counted from the bottom (1, the lowest, makes the crutch longest)
OCT_TO = 0.300  # the main tube is hollow (an octagon of wall panels) up to here; solid above, where no leg reaches
Y_GRIP = 0.360  # the handgrip ...
GRIP_R, GRIP_L = 0.015, 0.115  # ... 3 cm across, sticking out 11.5 cm to the front
Y_SHAFT_END = 0.560  # the main tube ends at the cuff
CUFF_Y0, CUFF_Y1 = 0.555, 0.615
CUFF_R, CUFF_DOWN = 0.045, 0.050  # the cuff wraps a 9 cm forearm whose axis is 5 cm in front of the tube's
COLLAR_R, COLLAR_Y = 0.0135, (-0.002, 0.022)  # the plastic collar on the main tube's lower end
CRUTCH_MASS = 0.40

# the leg (inner tube), in its own frame: origin on its axis where its spring button is; +y toward its top end
LEG_TOP = 0.030  # the leg's top end, 3 cm past the button
LEG_LEN = 0.580
TIP_R, TIP_L = 0.020, 0.045  # the rubber tip, over the leg's last 1 cm ...
TIP_PAST = TIP_L - 0.010  # ... so it reaches 3.5 cm past the leg's end
LEG_MASS = 0.14

# the spring button, in its own frame: origin at its dome's centre
BUTTON_R = 0.0032  # 6.4 mm (1/4 in) across, a domed head on a straight shank
PROUD = 0.0040  # let out, its top stands 4 mm out of the main tube
BUTTON_Z = R_OUT + PROUD - BUTTON_R  # 11.9 mm: the dome's centre above the axis, let out
SHANK_FROM = 0.0040  # the shank runs down to 4 mm above the axis (its spring clip is inside the leg)
BUTTON_TRAVEL = 0.0060
BUTTON_K, BUTTON_PRE = 800.0, 0.0060  # N/m and preload: 4.8 N let out, 9 N pressed down inside the main tube
BUTTON_MASS = 0.006

# a let-out button's straight shank fills the hole through the whole wall; the leg slides only once the dome's centre
# is below the main tube's inside, i.e. the button is pressed more than 2 mm of its 4 mm, nearly flush
UNLOCK_PRESS = BUTTON_Z - R_INNER


def hole_y(k: int) -> float:
    """Hole k's middle, along the crutch from the main tube's lower end (k = 1 the lowest)."""
    return HOLE1_Y + (k - 1) * PITCH


def length_at(k: int) -> float:
    """The crutch's handle height (tip to handgrip) with the button in hole k."""
    exposed = LEG_LEN - LEG_TOP - hole_y(k)  # the leg's length below the main tube's end
    return Y_GRIP + exposed + TIP_PAST


# -- the layout ---------------------------------------------------------------------------------------------------------
AXIS_UP = 0.135  # the crutches' axes this far above the table: the handgrips hang down with 1 cm to spare
C_Y = 0.000  # the crutches' lower (collar) ends at this world y; their tips to the right (-y), cuffs to the left
CRUTCHES = {  # name -> (world x of its axis, the hole its button starts in)
    "near": (0.230, 2),  # set three holes too long
    "far": (0.350, 8),  # set three holes too short
}
CRADLE_YS = (0.010, 0.470)  # the stand's cradles, along the crutch: one under the collar, one between handgrip and cuff


def crutch_origin(name: str) -> np.ndarray:
    return np.array([CRUTCHES[name][0], C_Y, TOP + AXIS_UP])


def _oct_wall(y0: float, y1: float, color, skip_top: bool) -> list:
    """The main tube's wall from y0 to y1: eight flat panels round the axis (the top one left out if ``skip_top``)."""
    r_mid = R_OUT - WALL / 2
    width = 2 * r_mid * math.tan(math.pi / 8) + 0.0010  # a little overlap at the corners
    out = []
    for j in range(8):
        a = math.pi / 2 + j * math.pi / 4
        if skip_top and j == 0:
            continue
        n = np.array([math.cos(a), 0.0, math.sin(a)])
        c = n * r_mid
        out.append(plate(c + [0.0, y0, 0.0], c + [0.0, y1, 0.0], width, WALL, n, color))
    return out


def _top_panels() -> list:
    """The main tube's top (back) panel, from its end to OCT_TO, broken by the nine holes."""
    r_mid = R_OUT - WALL / 2
    width = 2 * r_mid * math.tan(math.pi / 8) + 0.0010
    edges = [0.0]
    for k in range(1, N_HOLES + 1):
        edges += [hole_y(k) - HOLE_W / 2, hole_y(k) + HOLE_W / 2]
    edges.append(OCT_TO)
    c = np.array([0.0, 0.0, r_mid])
    return [plate(c + [0.0, edges[i], 0.0], c + [0.0, edges[i + 1], 0.0], width, WALL, (0.0, 0.0, 1.0), ALU)
            for i in range(0, len(edges), 2)]


def _red_bands() -> list:
    """Two thin red tape bands round the main tube, either side of the target hole (visual only)."""
    r = R_OUT + 0.0003
    width = 2 * r * math.tan(math.pi / 8) + 0.0010
    out = []
    for y in (hole_y(TARGET) - HOLE_W / 2 - 0.0025, hole_y(TARGET) + HOLE_W / 2 + 0.0005):
        for j in range(8):
            a = math.pi / 2 + j * math.pi / 4
            n = np.array([math.cos(a), 0.0, math.sin(a)])
            out.append(plate(n * r + [0.0, y, 0.0], n * r + [0.0, y + 0.002, 0.0], width, 0.0004, n, RED,
                             collide=False))
    return out


def _cuff() -> list:
    """An open cuff: a 240-degree band round a forearm in front of (below) the tube, open toward the front."""
    out, n_seg = [], 8
    a0, a1 = math.radians(-30.0), math.radians(210.0)
    for i in range(n_seg):
        a = a0 + (a1 - a0) * (i + 0.5) / n_seg
        n = np.array([math.cos(a), 0.0, math.sin(a)])
        c = np.array([0.0, 0.0, -CUFF_DOWN]) + n * CUFF_R
        w = 2 * CUFF_R * math.tan((a1 - a0) / n_seg / 2) + 0.002
        out.append(plate(c + [0.0, CUFF_Y0, 0.0], c + [0.0, CUFF_Y1, 0.0], w, 0.003, n, CUFF_C))
    return out


def crutch_shapes() -> list:
    shapes = _oct_wall(0.0, OCT_TO, ALU, skip_top=True) + _top_panels() + _red_bands()
    shapes += [
        CylShape((0.0, (OCT_TO + Y_SHAFT_END) / 2, 0.0), R_OUT, Y_SHAFT_END - OCT_TO, ALU, axis="Y"),
        CylShape((0.0, sum(COLLAR_Y) / 2, 0.0), COLLAR_R, COLLAR_Y[1] - COLLAR_Y[0], BLACK, axis="Y"),
        BoxShape((0.0, Y_GRIP, -0.014), (0.024, 0.030, 0.012), CUFF_C),  # the handgrip's socket
        CylShape((0.0, Y_GRIP, -0.010 - GRIP_L / 2), GRIP_R, GRIP_L, GRIP_C, axis="Z"),
        BoxShape((0.0, (CUFF_Y0 + CUFF_Y1) / 2, -0.004), (0.020, CUFF_Y1 - CUFF_Y0, 0.010), CUFF_C),  # cuff mount
        *_cuff(),
    ]
    return shapes


def leg_shapes() -> list:
    bottom = LEG_TOP - LEG_LEN
    return [CylShape((0.0, (LEG_TOP + bottom) / 2, 0.0), LEG_R, LEG_LEN, LEG_C, axis="Y"),
            CylShape((0.0, bottom + 0.010 - TIP_L / 2, 0.0), TIP_R, TIP_L, BLACK, axis="Y")]


def button_shapes() -> list:
    shank = BUTTON_Z - SHANK_FROM
    return [SphereShape((0.0, 0.0, 0.0), BUTTON_R, BUTTON_C),
            CylShape((0.0, 0.0, -shank / 2), BUTTON_R, shank, BUTTON_C, axis="Z")]


def _stand() -> Body:
    """A fixed wooden stand: under each crutch a cradle (a block with a post either side) beneath the collar and
    another beneath the tube between the handgrip and the cuff."""
    shapes = []
    for name, (x, _) in CRUTCHES.items():
        for cy, r in zip(CRADLE_YS, (COLLAR_R, R_OUT)):
            base = AXIS_UP - r
            y = C_Y + cy
            shapes.append(BoxShape((x - 0.30, y, base / 2), (0.040, 0.020, base), WOOD))
            for s in (-1.0, 1.0):
                shapes.append(BoxShape((x - 0.30 + s * (r + 0.0045), y, base + 0.005), (0.006, 0.020, 0.010), WOOD))
    return Body.build("stand", pos=(0.30, 0.0, TOP), mass=2.0, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=WOOD)


def _bodies(name: str) -> list[Body]:
    x, k0 = CRUTCHES[name]
    o = crutch_origin(name)
    button_at = o + np.array([0.0, hole_y(k0), 0.0])
    crutch = Body.build(f"crutch_{name}", pos=tuple(o.tolist()), mass=CRUTCH_MASS, shapes=crutch_shapes(), color=ALU,
                        friction=(0.6, 0.5), com=(0.0, 0.30, -0.015), contact_offset=0.0008, solver_iterations=32)
    leg = Body.build(f"leg_{name}", pos=tuple(button_at.tolist()), mass=LEG_MASS, shapes=leg_shapes(), color=LEG_C,
                     friction=(0.6, 0.5), ignore=(crutch.name,), contact_offset=0.0008, solver_iterations=32)
    button = Body.build(f"button_{name}", pos=tuple((button_at + [0.0, 0.0, BUTTON_Z]).tolist()), mass=BUTTON_MASS,
                        shapes=button_shapes(), color=BUTTON_C, friction=(0.4, 0.3), ignore=(leg.name,),
                        contact_offset=0.0005, solver_iterations=32)
    # the leg slides along the crutch, from the lowest hole to the highest and a little past
    slider(f"slide_{name}", crutch.name, leg.name, anchor=tuple(button_at.tolist()), axis="Y",
           limits=(hole_y(1) - hole_y(k0) - 0.006, hole_y(N_HOLES) - hole_y(k0) + 0.006), damping=2.0)
    # the button moves in and out of the leg, sprung outward
    slider(f"spring_{name}", leg.name, button.name, anchor=tuple((button_at + [0.0, 0.0, BUTTON_Z]).tolist()), axis="Z",
           limits=(-BUTTON_TRAVEL, 0.0), stiffness=BUTTON_K, damping=2.0, target=BUTTON_PRE, max_force=100.0)
    return [crutch, leg, button]


STAND = _stand()
BODIES = {n: _bodies(n) for n in CRUTCHES}
OBJECTS = [STAND, *[b for n in CRUTCHES for b in BODIES[n]]]

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_HOLE = 0.0015  # the button's middle within 1.5 mm of the red hole's middle, along the tube ...
FULL_OUT = 0.0030  # ... and its top at least 3 mm out of the tube (4 mm when fully out): it is through the hole
LET_GO = 0.040  # every gripper TCP at least 4 cm from both crutches' axes (tip to cuff)
ON_STAND_Z = 0.010  # a crutch is on its stand: its axis within 1 cm of where it started ...
ON_STAND_ROLL = 20.0  # ... and its handgrip within 20 degrees of straight down
DROPPED_Z = 0.040  # a crutch whose axis ever drops 4 cm, or turns over (roll beyond 60 degrees), was dropped
DROPPED_ROLL = 60.0
PER_CRUTCH = 0.4


def _fresh() -> dict:
    return {"dropped": False, "tcps": None, "steps": 0}


_EP = _fresh()
_DEBUG = bool(os.environ.get("CRUTCH_DEBUG"))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def roll_deg(rot: np.ndarray) -> float:
    """How far the crutch has turned about its own axis: the angle of its body z (its back) from straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def button_reading(c_pos, c_rot, b_pos) -> tuple[float, float]:
    """(the button's middle along the crutch from the main tube's end, how far its top stands out of the tube) (m)."""
    local = c_rot.T @ (np.asarray(b_pos, float) - np.asarray(c_pos, float))
    return float(local[1]), float(local[2] + BUTTON_R - R_OUT)


def is_set(c_pos, c_rot, b_pos) -> bool:
    y, out = button_reading(c_pos, c_rot, b_pos)
    return abs(y - hole_y(TARGET)) <= IN_HOLE and out >= FULL_OUT


def axis_distance(c_pos, c_rot, l_pos, point) -> float:
    """A point's distance from the crutch's axis, between its rubber tip's end and its cuff."""
    p = c_rot.T @ (np.asarray(point, float) - np.asarray(c_pos, float))
    tip_end = float((c_rot.T @ (np.asarray(l_pos, float) - np.asarray(c_pos, float)))[1]) + LEG_TOP - LEG_LEN - TIP_PAST
    y = min(max(float(p[1]), tip_end), CUFF_Y1)
    return float(np.linalg.norm(p - np.array([0.0, y, 0.0])))


def on_stand(name: str, c_pos, c_rot) -> bool:
    return abs(float(c_pos[2]) - float(crutch_origin(name)[2])) <= ON_STAND_Z and roll_deg(c_rot) <= ON_STAND_ROLL


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    try:
        _EP["tcps"] = _tcps(base, origin)
    except (KeyError, StopIteration, AttributeError):
        _EP["tcps"] = None
    lines = []
    for name in CRUTCHES:
        c = as_numpy(base.scene[f"crutch_{name}"].data.root_link_pose_w)[0]
        c_pos, c_rot = c[:3] - origin, rotation(c[3:7])
        if float(c_pos[2]) < float(crutch_origin(name)[2]) - DROPPED_Z or roll_deg(c_rot) > DROPPED_ROLL:
            _EP["dropped"] = True
        if _DEBUG and _EP["steps"] % 5 == 0:
            b = as_numpy(base.scene[f"button_{name}"].data.root_link_pose_w)[0]
            y, out = button_reading(c_pos, c_rot, b[:3] - origin)
            lines.append(f"{name}: hole {1 + (y - HOLE1_Y) / PITCH:5.2f} out {out * 1000:5.2f}mm "
                         f"crutch {np.round((c_pos - crutch_origin(name)) * 1000, 1).tolist()}mm "
                         f"roll {roll_deg(c_rot):4.1f} set {is_set(c_pos, c_rot, b[:3] - origin)}")
    if lines:
        tc = [np.round(t, 3).tolist() for t in _EP["tcps"]] if _EP["tcps"] is not None else None
        print(f"[crutch] step {_EP['steps']} dropped {_EP['dropped']} tcps {tc} | " + " | ".join(lines),
              file=sys.stderr, flush=True)


def status(objects) -> dict:
    out = {"set": [], "on_stand": True, "let_go": True, "still": True}
    tcps = _EP["tcps"]
    for name in CRUTCHES:
        c, leg, b = objects[f"crutch_{name}"], objects[f"leg_{name}"], objects[f"button_{name}"]
        c_pos, c_rot = np.asarray(c.pos, float), rotation(c.quat_xyzw)
        if is_set(c_pos, c_rot, b.pos):
            out["set"].append(name)
        out["on_stand"] &= on_stand(name, c_pos, c_rot)
        if tcps is None:
            out["let_go"] = False
        else:
            out["let_go"] &= all(axis_distance(c_pos, c_rot, leg.pos, t) >= LET_GO for t in tcps)
        out["still"] &= c.resting() and leg.resting()
    return out


def points(st: dict, dropped: bool) -> float:
    if dropped:
        return 0.0
    n = len(st["set"])
    if n == len(CRUTCHES) and st["let_go"] and st["on_stand"] and st["still"]:
        return 1.0
    return PER_CRUTCH * n


def score(objects) -> float:
    """0.4 for each crutch whose button stands fully out (its top at least 3 mm out of the tube) through the hole ringed
    in red; 1.0 with both, both crutches let go (every gripper TCP 4 cm from both), on their stand and still. 0 if a
    crutch was dropped (its axis 4 cm below where it lay, or turned over)."""
    return points(status(objects), _EP["dropped"])


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------
# For each crutch in turn the right hand pinches the leg below the main tube, and the left presses the leg's button
# flush with closed fingertips. The right slides the leg along the tube while the left's fingertips glide along with the
# button, just clear of the tube, keeping it pressed (otherwise it would spring out into the next hole it passes). Over
# the red hole the left lifts away and the button springs out through it; then the right lets go.

TIP = 0.013  # the pad tips this far below the TCP
PRESS_GAP = float(os.environ.get("CRUTCH_PRESS_GAP", "0.0005"))  # pad tips this far above the tube while pressing
# the arms miss their commanded poses by a pose-dependent offset; aim the other way. Tune on Brev:
# CRUTCH_SAG_L / CRUTCH_SAG_R = "dx,dy,dz" (m)
SAG_L = np.array([float(v) for v in os.environ.get("CRUTCH_SAG_L", "0,0,0").split(",")])
SAG_R = np.array([float(v) for v in os.environ.get("CRUTCH_SAG_R", "0,0,0").split(",")])
PRESS_OFF = np.array([float(v) for v in os.environ.get("CRUTCH_PRESS_OFF", "0,0").split(",")] + [0.0])
GRIP_UP = 0.004  # the right's TCP this far above the leg's axis while pinching it
YAW = math.pi / 2  # the right's grip: straight down, fingers closing across the leg (along x)
YAW_PRESS = 0.0  # the left presses with closed fingertips turned square to the tube (the wrist reaches it there)
HOVER_L = 0.030  # the left comes down from just above the button (higher is out of its reach so far inward)
HOVER = 0.035  # the right comes down from this far above the leg (its pad tips then 1.6 cm over the tube)
SLIDE_S_PER_HOLE = 1.3
# where the right pinches the leg, along the crutch from the main tube's end: far enough out that the fingers stay
# clear of the collar and the cradle under it however far the leg slides in
GRIP_LEG_Y = {"near": -0.130, "far": -0.085}
PARK = {"left": (0.15, 0.30, TOP + 0.20), "right": (0.15, -0.30, TOP + 0.20)}


def _w(t, pos, finger, label, yaw=YAW) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in pos), finger, label, yaw)


def _wl(t, pos, finger, label) -> Waypoint:
    return _w(t, pos, finger, label, YAW_PRESS)


def press_point(name: str, k: float) -> np.ndarray:
    """The left's TCP pressing the button flush when the button is at hole position k (fractional allowed)."""
    o = crutch_origin(name)
    return o + np.array([0.0, hole_y(1) + (k - 1) * PITCH, R_OUT + PRESS_GAP + TIP]) + PRESS_OFF + SAG_L


def grip_point(name: str, slid: float) -> np.ndarray:
    """The right's TCP pinching the leg, the leg slid ``slid`` (m, +y) from where it started."""
    o = crutch_origin(name)
    return o + np.array([0.0, GRIP_LEG_Y[name] + slid, GRIP_UP]) + SAG_R


def plan_crutch(name: str, stop_short: float = 0.0, press: bool = True, release_right: bool = True
                ) -> tuple[list[Waypoint], list[Waypoint]]:
    """(left, right) waypoints adjusting one crutch from its starting hole to the red one, both starting from above."""
    k0 = CRUTCHES[name][1]
    slide = (hole_y(TARGET) - hole_y(k0)) - math.copysign(stop_short, hole_y(TARGET) - hole_y(k0))
    k1 = k0 + slide / PITCH
    t_slide = SLIDE_S_PER_HOLE * abs(TARGET - k0)
    up = np.array([0.0, 0.0, HOVER])
    g0, g1 = grip_point(name, 0.0), grip_point(name, slide)
    p0, p1 = press_point(name, k0), press_point(name, k1)
    right = [_w(2.0, g0 + up, FINGER_OPEN, f"{name}: over the leg"),
             _w(1.2, g0, FINGER_OPEN, f"{name}: down round the leg"),
             _w(0.8, g0, FINGER_CLOSED, f"{name}: pinch the leg")]
    left = [_wl(2.0, p0 + [0.0, 0.0, HOVER_L], FINGER_CLOSED, f"{name}: over the button"),
            _wl(1.2, p0 + [0.0, 0.0, 0.008], FINGER_CLOSED, f"{name}: down to the button")]
    if press:
        left.append(_wl(0.8, p0, FINGER_CLOSED, f"{name}: press it flush"))
    else:
        left.append(_wl(0.8, p0 + [0.0, 0.0, 0.008], FINGER_CLOSED, f"{name}: rest over it, not pressing"))
    # both arms reach these poses together: right after 4.0 s, left after 4.0 s
    left.append(_wl(0.4, left[-1].pos, FINGER_CLOSED, f"{name}: hold it pressed"))
    right.append(_w(0.4, g0, FINGER_CLOSED, f"{name}: wait for the press"))
    if press:
        left.append(_wl(t_slide, p1, FINGER_CLOSED, f"{name}: glide along with the button"))
    else:
        left.append(_wl(t_slide, left[-1].pos, FINGER_CLOSED, f"{name}: stay over it"))
    right.append(_w(t_slide, g1, FINGER_CLOSED, f"{name}: slide the leg"))
    lifted = np.array(left[-1].pos) + [0.0, 0.0, 0.03]
    left += [_wl(0.8, lifted, FINGER_CLOSED, f"{name}: lift off: the button springs out"),
             _wl(0.8, lifted, FINGER_CLOSED, f"{name}: wait")]
    right.append(_w(1.2, g1, FINGER_CLOSED, f"{name}: hold while it springs out"))
    if release_right:
        right += [_w(0.4, g1, FINGER_OPEN, f"{name}: let go of the leg"),
                  _w(0.8, g1 + up, FINGER_OPEN, f"{name}: up")]
    else:
        right += [_w(1.2, g1, FINGER_CLOSED, f"{name}: keep holding")]
    _pad(left, right)
    return left, right


def _length(wps) -> float:
    return sum(w.duration for w in wps)


def _pad(a: list, b: list) -> None:
    """Wait at the end of the shorter list so both last equally long."""
    la, lb = _length(a), _length(b)
    if la < lb:
        a.append(_w(lb - la, a[-1].pos, a[-1].finger, "wait", a[-1].yaw))
    elif lb < la:
        b.append(_w(la - lb, b[-1].pos, b[-1].finger, "wait", b[-1].yaw))


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        return {"left": [_wl(3.0, PARK["left"], FINGER_OPEN, "park")],
                "right": [_w(3.0, PARK["right"], FINGER_OPEN, "park")]}
    order = ["near", "far"]
    kw = {name: {} for name in order}
    if variant == "near_only":
        order = ["near"]
    if variant == "probe_far":  # Brev probe: the far crutch only (pull out)
        order = ["far"]
    if variant == "probe_near":  # Brev probe: the near crutch only (push in)
        order = ["near"]
    if variant == "no_press":  # the right pulls and pushes the legs without anyone pressing the buttons
        kw = {name: {"press": False} for name in order}
    if variant == "half_way":  # the far leg is left 6 mm short of the red hole: the button can't spring out
        kw["far"] = {"stop_short": 0.006}
    if variant == "keep_holding":  # both set, but the right never lets go of the last leg
        kw["far"] = {"release_right": False}
    left, right = [], []
    for name in order:
        lw, rw = plan_crutch(name, **kw[name])
        left += lw
        right += rw
    left.append(_wl(1.5, PARK["left"], FINGER_OPEN, "park"))
    if variant == "keep_holding":
        right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "keep holding the leg"))
        left.append(_wl(1.5, PARK["left"], FINGER_OPEN, "wait"))
    else:
        right.append(_w(1.5, PARK["right"], FINGER_OPEN, "park"))
    _pad(left, right)
    for arm in (left, right):
        arm.append(_w(2.0, arm[-1].pos, arm[-1].finger, "wait: the trial ends once the full score has held 1 s",
                       arm[-1].yaw))
    return {"left": left, "right": right}

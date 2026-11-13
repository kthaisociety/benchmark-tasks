"""On the station table a framed picture lies face down on two carpet-covered blocks, ready to be packed for a move:
an oak frame 30 cm front to back and 40 cm across, 2.5 cm thick, with its glass, mount and hardboard back. The blocks
run front to back under its two 30 cm sides, so all four corners stick out past them. Beside the frame lie four white
polyethylene-foam corner protectors, two on each side, each one beside a corner and already turned to face it.

A protector is a foam block 7 cm square and 4.5 cm tall with an L-shaped pocket cut into two adjacent sides: 4.5 cm
deep each way and as tall as the frame is thick. Its foam squeezes the frame, so it has to be pushed onto the corner,
level with the frame, until the corner reaches the back of the pocket. Pushed on, it grips the corner by itself.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import _SPECS, _part_state, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the four foam corner protectors onto the corners of the picture frame, then let go of them."

OAK = (0.62, 0.45, 0.27)
KRAFT = (0.55, 0.42, 0.30)
CARPET = (0.32, 0.33, 0.36)
FOAM = (0.93, 0.94, 0.95)

# -- the padded blocks: two carpet-covered pine 2x4 offcuts, fixed, under the frame's 30 cm sides ---------------------
BLOCK = (0.160, 0.089, 0.038)  # x (front to back), y, height
BLOCK_X = 0.30
BLOCK_Y = 0.185  # centred under each 30 cm side's moulding

# -- the picture frame, face down; origin at the middle of its front (the face on the blocks) --------------------------
FRAME_AT = (0.30, 0.0)
OUT = (0.30, 0.40)  # outside, x by y
MOULD_W, THICK = 0.030, 0.025  # moulding width; the frame's thickness, front to back
SLAB_Z = (0.004, 0.020)  # glass, mount, picture and hardboard back between the moulding's front lip and its back
FRAME_MASS = 1.1
FB = TOP + BLOCK[2]  # the frame's front (its underside) rests on the blocks at this height
HALF = (OUT[0] / 2, OUT[1] / 2)


def _frame():
    hx, hy = HALF
    w, t = MOULD_W, THICK
    rails = [((0.0, s * (hy - w / 2), t / 2), (OUT[0], w, t), OAK) for s in (-1, 1)]
    rails += [((s * (hx - w / 2), 0.0, t / 2), (w, OUT[1] - 2 * w, t), OAK) for s in (-1, 1)]
    slab = [((0.0, 0.0, sum(SLAB_Z) / 2), (OUT[0] - 2 * w + 0.002, OUT[1] - 2 * w + 0.002, SLAB_Z[1] - SLAB_Z[0]),
             KRAFT)]
    return part("frame", pos=(FRAME_AT[0], FRAME_AT[1], FB + 0.0005), mass=FRAME_MASS, color=OAK, friction=0.6,
                boxes=rails + slab)


FRAME = _frame()

# -- a foam corner protector, in its own frame: the origin is where the frame's corner sits, on the frame's front plane;
# the pocket opens toward -x and -y and the two closed walls are on +x and +y -----------------------------------------
POCKET = 0.045  # how far the corner goes in, each way
WALL = 0.025  # closed walls' thickness: the block is POCKET + WALL = 7 cm square
LIP = 0.010  # foam under and over the pocket
CLEAR = 0.0005  # the pocket's roof sits this far above the frame's back with the floor pressed down
PROUD = 0.002  # unpressed, the foam floor stands this far above the frame's front: it squeezes the frame 1.5 mm
RAMP_RUN, RAMP_RISE = 0.010, 0.006  # the pocket's open edges are bevelled: 6 mm over the outer 10 mm
ROOF = THICK + CLEAR
FLOOR_RATE = 4000.0  # N/m: about 6 N of squeeze at 1.5 mm
FLOOR_TRAVEL = 0.005
PROTECTOR_MASS = 0.020  # cross-linked PE foam, about 100 kg/m3
FOAM_FRICTION = 0.6


def _slope(axis: str, s_out: float, z_out: float, s_in: float, z_in: float, span: tuple[float, float],
           t: float, below: bool, color) -> tuple:
    """A thin box whose top (below=True) or bottom face runs from (s_out, z_out) to (s_in, z_in) along ``axis``
    ("x" or "y") and spans ``span`` along the other horizontal axis: a bevel on the pocket's open edge."""
    ds, dz = s_in - s_out, z_in - z_out
    length = math.hypot(ds, dz)
    ang = math.atan2(dz, ds)  # the slope's rise toward the inside
    us, uz = ds / length, dz / length
    ns, nz = -uz, us  # its upward normal
    k = -t / 2 if below else t / 2
    ms, mz = (s_out + s_in) / 2 + k * ns, (z_out + z_in) / 2 + k * nz
    mid_other, width = (span[0] + span[1]) / 2, span[1] - span[0]
    if axis == "x":  # turn about y: local x -> (cos a, 0, sin a) needs a rotation of -a
        q = (math.cos(-ang / 2), 0.0, math.sin(-ang / 2), 0.0)
        return ((ms, mid_other, mz), (length, width, t), color, q)
    q = (math.cos(ang / 2), math.sin(ang / 2), 0.0, 0.0)  # turn about x: local y -> (0, cos a, sin a)
    return ((mid_other, ms, mz), (width, length, t), color, q)


def _protector_shapes():
    p, w, lip, r, h = POCKET, WALL, LIP, RAMP_RUN, RAMP_RISE
    z0, z1 = -lip, ROOF + lip
    walls = [
        ((w / 2, (w - p) / 2, (z0 + z1) / 2), (w, p + w, z1 - z0), FOAM),  # wall on +x, full length
        ((-p / 2, w / 2, (z0 + z1) / 2), (p, w, z1 - z0), FOAM),  # wall on +y
    ]
    flat = p - r
    roof = [
        ((-p / 2, -p / 2, ROOF + h + (lip - h) / 2), (p, p, lip - h), FOAM),  # the top skin
        ((-flat / 2, -flat / 2, ROOF + h / 2), (flat, flat, h), FOAM),  # the roof's flat
        _slope("x", -p, ROOF + h, -flat, ROOF, (-p, 0.0), 0.004, False, FOAM),  # bevel on the -x opening
        _slope("y", -p, ROOF + h, -flat, ROOF, (-p, 0.0), 0.004, False, FOAM),  # bevel on the -y opening
    ]
    # the sprung floor, in the slider's frame (rest = PROUD above the origin): its top is the pocket's floor
    fz0 = -(lip - PROUD) + 0.002  # 2 mm above the walls' bottom, so it never props the block up
    floor = [
        ((-flat / 2, -flat / 2, fz0 / 2), (flat, flat, -fz0), FOAM),
        ((-p / 2, -p / 2, (fz0 + fz0 + 0.002) / 2), (p, p, 0.002), FOAM),  # under the bevels
        _slope("x", -p, -h, -flat, 0.0, (-p, 0.0), 0.004, True, FOAM),
        _slope("y", -p, -h, -flat, 0.0, (-p, 0.0), 0.004, True, FOAM),
    ]
    # partkit takes a slider's boxes in the part's frame at rest: lift the floor by its rest height
    floor = [((c[0], c[1], c[2] + PROUD), *rest) for c, *rest in floor]
    return walls + roof, floor


# corner name -> (sign along x, sign along y): the frame's corner at (sx * 15, sy * 20) cm from its middle
CORNERS = {"far_left": (1, 1), "near_left": (-1, 1), "far_right": (1, -1), "near_right": (-1, -1)}
# the protector's yaw that turns its pocket (open toward its -x, -y) onto that corner
YAW = {"far_left": 0.0, "near_left": math.pi / 2, "far_right": -math.pi / 2, "near_right": math.pi}
START_OUT = 0.100  # each protector starts on the table this far out sideways (y) from its seated place


def _rot(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def corner_xy(name: str) -> np.ndarray:
    sx, sy = CORNERS[name]
    return np.array([FRAME_AT[0] + sx * HALF[0], FRAME_AT[1] + sy * HALF[1]])


def start_xy(name: str) -> np.ndarray:
    return corner_xy(name) + np.array([0.0, CORNERS[name][1] * START_OUT])


def _protector(name: str):
    shapes, floor = _protector_shapes()
    x, y = start_xy(name)
    slider = Slider(f"floor_{name}", rest=(0.0, 0.0, PROUD), boxes=tuple(floor), travel=FLOOR_TRAVEL,
                    rate=FLOOR_RATE, damping=8.0, mass=0.004, friction=FOAM_FRICTION)
    return part(f"protector_{name}", pos=(x, y, TOP + LIP + 0.0005), boxes=shapes, mass=PROTECTOR_MASS,
                yaw=YAW[name], color=FOAM, friction=FOAM_FRICTION, sliders=(slider,))


PROTECTORS = {n: _protector(n) for n in CORNERS}

OBJECTS = [
    *[Box(f"block_{side}", pos=(BLOCK_X, s * BLOCK_Y, TOP + BLOCK[2] / 2), size=BLOCK, color=CARPET)
      for side, s in (("left", 1), ("right", -1))],
    FRAME,
    *PROTECTORS.values(),
]

# -- scoring -----------------------------------------------------------------------------------------------------------
IN_TOL = 0.004  # the corner within 4 mm of the back of the pocket, both ways
OVER_TOL = 0.002  # (the corner can't go into the wall: allow for contact)
Z_TOL = 0.003  # the frame's front within 3 mm of the pocket floor's line
TURN_TOL = 6.0  # degrees: the protector square to the corner
TILT_TOL = 6.0  # degrees: level with the frame
SQUEEZE = 0.0008  # the floor pressed down at least this far: the foam grips the frame
LET_GO = 0.04  # each gripper's TCP at least this far from the protector's middle
DROPPED_TILT = 15.0  # the frame ever tilted more than this, or fell off a block: dropped
FLAT_TILT = 3.0  # at the end the frame lies flat on both blocks
OFF_BLOCK = 0.015  # its front this far below the blocks' top: it slid off a block


def _fresh() -> dict:
    return {"dropped": False, "steps": 0, "tcps": []}


_S = _fresh()
_DEBUG = bool(os.environ.get("CORNERS_DEBUG"))


def _tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    c = (np.trace(a.T @ b) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(c)))))


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def corner_world(frame, name: str) -> np.ndarray:
    sx, sy = CORNERS[name]
    return frame.to_world((sx * HALF[0], sy * HALF[1], 0.0))


def protector_middle(prot) -> np.ndarray:
    return prot.to_world(((WALL - POCKET) / 2, (WALL - POCKET) / 2, ROOF / 2))


def squeeze(prot) -> float:
    """How far the protector's foam floor is pressed down (m, positive)."""
    return -min(0.0, float(next(iter(prot.sliders.values()), 0.0))) if prot.sliders else 0.0


def seated_on(frame, prot, name: str) -> bool:
    """The protector is fully on corner ``name``: the corner at the back of its pocket, square and level with the
    frame, and the foam floor pressed down so it grips."""
    local = prot.to_local(corner_world(frame, name))
    if not (-IN_TOL <= local[0] <= OVER_TOL and -IN_TOL <= local[1] <= OVER_TOL and abs(local[2]) <= Z_TOL):
        return False
    want = frame.rotation() @ _rz(YAW[name])
    return _angle_deg(want, prot.rotation()) <= max(TURN_TOL, TILT_TOL) and squeeze(prot) >= SQUEEZE


def released(prot) -> bool:
    mid = protector_middle(prot)
    return bool(_S["tcps"]) and all(np.linalg.norm(np.asarray(t) - mid) >= LET_GO for t in _S["tcps"])


def frame_flat(frame) -> bool:
    return _tilt_deg(frame) <= FLAT_TILT and float(frame.pos[2]) > FB - OFF_BLOCK


def protected(objects) -> list[str]:
    """The corners that have a protector seated on them, let go."""
    frame = objects[FRAME.name]
    out = []
    for name in CORNERS:
        for spec in PROTECTORS.values():
            prot = objects[spec.name]
            if seated_on(frame, prot, name) and released(prot):
                out.append(name)
                break
    return out


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips)."""
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


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(_fresh())


@on_step
def _follow(environment) -> None:
    _S["steps"] += 1
    frame = _part_state(environment, _SPECS[FRAME.name])
    if _tilt_deg(frame) > DROPPED_TILT or float(frame.pos[2]) < FB - OFF_BLOCK:
        _S["dropped"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    if _DEBUG and _S["steps"] % 5 == 0:
        parts = []
        for name, spec in PROTECTORS.items():
            prot = _part_state(environment, _SPECS[spec.name])
            local = prot.to_local(corner_world(frame, name))
            parts.append(f"{name}: corner {np.round(local * 1000, 1).tolist()}mm squeeze {squeeze(prot) * 1000:.2f}mm "
                         f"seated {seated_on(frame, prot, name)}")
        print(f"[corners] step {_S['steps']} frame {np.round(frame.pos, 4)} tilt {_tilt_deg(frame):.1f} "
              f"dropped {_S['dropped']} tcps {[np.round(t, 3).tolist() for t in _S['tcps']]} | " + " | ".join(parts),
              flush=True)


def score(objects) -> float:
    frame = objects[FRAME.name]
    if _S["dropped"] or not frame_flat(frame):
        return 0.0
    n = len(protected(objects))
    if n < 4:
        return 0.25 * n
    still = frame.resting() and all(objects[s.name].resting() for s in PROTECTORS.values())
    return 1.0 if still else 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------
# Each arm fits the two protectors on its own side while the other arm holds the frame down on its back. A protector is
# pinched across its closed wall (fingers closing along world y), carried to the frame's height beside its corner, and
# pushed straight on sideways (along y); then the gripper opens and backs away.

TIP = 0.013  # pad tips below the TCP
PINCH_UP = 0.007  # TCP above the protector's origin while pinching it (pad tips 4 mm above the table at the start)
HOVER = TOP + 0.10
APPROACH = 0.050  # start the push this far out from seated, along y
X_CLEAR = 0.0015  # ... and this far out along x, so the closed wall clears the frame's end ...
X_PUSH = 0.002  # ... then press this far past seated along x and y at the end of the push
Z_AIM = -0.00125  # the protector's origin this far below the frame's front: the middle of the bevels' window
PRESS = 0.012  # the holding arm's closed fingertips aimed this far below the back's surface
HOLD_Y = 0.10  # the holding arm presses on the back this far either side of the middle
# the loaded arm misses its commanded pose by a pose-dependent offset (x, y, z); the demo aims the other way. Tune on
# Brev with CORNERS_SAG="dx,dy,dz".
SAG = tuple(float(v) for v in os.environ.get("CORNERS_SAG", "0,0,0").split(","))


def grip_local(name: str) -> np.ndarray:
    """Where the TCP pinches the protector, in its own frame: across the closed wall whose faces are square to y."""
    if abs(math.cos(YAW[name])) > 0.5:  # world y along the protector's y: pinch its +x wall
        return np.array([WALL / 2, (WALL - POCKET) / 2])
    return np.array([(WALL - POCKET) / 2, WALL / 2])  # world y along its x: pinch its +y wall


def grip_world(name: str, origin_xy) -> np.ndarray:
    return np.asarray(origin_xy, float) + _rot(YAW[name]) @ grip_local(name)


def _fit(name: str) -> list[Waypoint]:
    sx, sy = CORNERS[name]
    seat = corner_xy(name)
    g_start = grip_world(name, start_xy(name))
    g_pre = grip_world(name, seat + np.array([sx * X_CLEAR, sy * APPROACH]))
    g_in = grip_world(name, seat + np.array([sx * X_CLEAR, -sy * X_PUSH]))
    g_home = grip_world(name, seat + np.array([-sx * X_PUSH, -sy * X_PUSH]))
    z_pick = TOP + LIP + 0.0005 + PINCH_UP
    z_fit = FB + Z_AIM + PINCH_UP + SAG[2]
    dx, dy = SAG[0], SAG[1]

    def at(g, z):
        return (float(g[0] + dx), float(g[1] + dy), float(z))

    return [
        Waypoint(1.8, (float(g_start[0]), float(g_start[1]), HOVER), FINGER_OPEN, f"{name}: above the protector"),
        Waypoint(1.4, (float(g_start[0]), float(g_start[1]), z_pick), FINGER_OPEN, f"{name}: down round its wall"),
        Waypoint(0.8, (float(g_start[0]), float(g_start[1]), z_pick), FINGER_CLOSED, f"{name}: pinch"),
        Waypoint(1.0, (float(g_start[0]), float(g_start[1]), z_fit + 0.03), FINGER_CLOSED, f"{name}: lift"),
        Waypoint(1.4, at(g_pre, z_fit + 0.03), FINGER_CLOSED, f"{name}: beside the corner"),
        Waypoint(1.0, at(g_pre, z_fit), FINGER_CLOSED, f"{name}: down to the frame's height"),
        Waypoint(0.5, at(g_pre, z_fit), FINGER_CLOSED, f"{name}: steady"),
        Waypoint(3.0, at(g_in, z_fit), FINGER_CLOSED, f"{name}: push it on sideways"),
        Waypoint(1.0, at(g_home, z_fit), FINGER_CLOSED, f"{name}: corner into the back of the pocket"),
        Waypoint(0.6, at(g_home, z_fit), FINGER_CLOSED, f"{name}: settle"),
        Waypoint(0.6, at(g_home, z_fit), FINGER_OPEN, f"{name}: let go"),
        Waypoint(0.8, at(g_home, z_fit + 0.05), FINGER_OPEN, f"{name}: up"),
    ]


def _hold(side: str, duration: float) -> list[Waypoint]:
    """Closed fingers pressed down on the frame's back, beside the middle on ``side``, for ``duration`` s in all."""
    y = HOLD_Y if side == "left" else -HOLD_Y
    x = FRAME_AT[0]
    z_back = FB + SLAB_Z[1]
    z = z_back + TIP - PRESS
    down, up = 1.4, 0.8
    return [
        Waypoint(1.6, (x, y, HOVER), FINGER_CLOSED, f"{side}: above the frame's back"),
        Waypoint(down, (x, y, z), FINGER_CLOSED, f"{side}: press the frame down"),
        Waypoint(max(0.1, duration - 1.6 - down - up), (x, y, z), FINGER_CLOSED, f"{side}: hold it"),
        Waypoint(up, (x, y, HOVER), FINGER_CLOSED, f"{side}: up off the frame"),
    ]


def _wait(duration: float, pos, finger=FINGER_OPEN, label="wait") -> list[Waypoint]:
    return [Waypoint(duration, pos, finger, label)] if duration > 0 else []


def _length(wps) -> float:
    return sum(w.duration for w in wps)


PARK = {"left": (0.18, 0.34, TOP + 0.16), "right": (0.18, -0.34, TOP + 0.16)}


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        return {"left": _wait(3.0, PARK["left"]), "right": _wait(3.0, PARK["right"])}
    left_fit = _fit("far_left") + _fit("near_left")
    right_fit = _fit("far_right") + _fit("near_right")
    if variant == "no_hold":  # each arm fits its own side's protectors, nobody holds the frame
        return {"left": left_fit + [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")],
                "right": _wait(_length(left_fit), PARK["right"]) + right_fit
                + [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park")]}
    if variant == "keep_holding":  # all four fitted, but the last one is never let go
        right_fit = right_fit[:-2] + [Waypoint(3.0, right_fit[-3].pos, FINGER_CLOSED, "keep holding")]
    if variant == "two_only":  # only the left side's two
        right_fit = []
    if variant == "probe_one":  # Brev probe: the far-left protector only, the right arm holding
        left_fit, right_fit = _fit("far_left"), []
    if variant == "probe_near":  # Brev probe: the near-left protector only (the reach close to the arm's base)
        left_fit, right_fit = _fit("near_left"), []
    # phase 1: left fits, right holds; phase 2: right fits, left holds
    t1 = _length(left_fit)
    left = left_fit + [Waypoint(1.0, (FRAME_AT[0], HOLD_Y, HOVER), FINGER_CLOSED, "left: over the frame")]
    right = _hold("right", t1 + 1.0)
    if right_fit:
        left += _hold("left", _length(right_fit) + 2.0)[1:] + [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")]
        right += right_fit + [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park")]
    else:
        left += [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")]
        right += [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park")]
    return {"left": left, "right": right}

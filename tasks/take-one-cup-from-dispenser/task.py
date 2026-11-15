"""On the station table, a counter set against a white wall: a stainless-steel pull-type cup dispenser hangs on the wall
in front of the left arm, its tube vertical and its outlet 15 cm above the counter. It holds a short stack of nested
brown kraft 12 oz paper hot cups, upside up (open end up). The bottom cup hangs out of the outlet: its rolled rim rests on
the dispenser's black silicone gasket, whose eight soft flaps hold it by the rim, and the rest of the cup (about 9 cm of
it) sticks out below the dispenser. The other cups sit nested inside it, up in the tube. A cup comes out when it is
pulled straight down hard enough for its rim to push the flaps aside; the flaps then spring back and catch the next
cup's rim. To the right of the dispenser, on the counter, lies a black rubber bar mat.

A paper cup is light and thin: it is held by pinching its side gently, and it can only come out straight down.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from partkit import _SPECS, _part_state, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Pull one paper cup out of the wall dispenser and stand it upright on the black mat."

PAPER = (0.62, 0.46, 0.30)  # unbleached kraft: stands out against the steel tube and the white wall
STEEL = (0.74, 0.75, 0.77)
SILICONE = (0.10, 0.10, 0.11)
WALL_WHITE = (0.90, 0.90, 0.88)
RUBBER = (0.07, 0.07, 0.08)

# -- a 12 oz (355 ml) single-wall paper hot cup, in its own frame: origin at the middle of its bottom edge, z up ---------
CUP_H = 0.110  # overall height, base to the top of the rolled rim
RIM_D = 0.090  # across the rolled rim
BASE_D = 0.060  # across the bottom of the wall
BEAD_R = 0.0016  # the rolled rim is a 3.2 mm roll ...
BEAD_RC = RIM_D / 2 - BEAD_R  # ... centred this far from the axis ...
BEAD_ZC = CUP_H - BEAD_R  # ... and this high
WALL_T = 0.0012  # the wall's thickness in the sim (paper and its coating are about 0.4 mm)
R0 = BASE_D / 2  # the wall's outside radius at the bottom ...
R_TOP = BEAD_RC - 0.25 * BEAD_R  # ... and where it meets the roll: the rim stands 2 mm proud of the wall
SLOPE = (R_TOP - R0) / BEAD_ZC  # the wall's taper: radius gained per metre of height
CUP_MASS = 0.011
CUP_FRICTION = 0.6
WALL_SEGS = 24  # flat facets round the wall (faces at every 15 degrees, so ones face +-x and +-y)
BEAD_SEGS = 24
BASE_Z = (0.0050, 0.0065)  # the recessed paper bottom
NEST_PITCH = 0.018  # nested cups sit this far apart: each one's bottom edge rests on four small stacking bumps
LUG_R = 0.0015
N_CUPS = 4  # cups in the dispenser


def wall_radius(z: float) -> float:
    """The cup wall's outside radius (m) at height ``z`` above its bottom edge."""
    return R0 + SLOPE * z


def _ring_prisms(sections, n: int, color, visual: bool = False, phase: float = 0.0) -> list:
    """Hull specs for a ring made of ``n`` convex prisms round the z axis. ``sections`` is a list of (rho, z) points (a
    convex cross-section in a half-plane); each prism spans one n-th of a turn, with its radii stretched by
    1/cos(half the span) so its flat faces touch the true circle at their middles. ``phase`` turns the ring."""
    half = math.pi / n
    grow = 1.0 / math.cos(half)
    out = []
    for k in range(n):
        mid = phase + 2 * half * k
        a, b = mid - half, mid + half
        sec_a = [(rho * grow * math.cos(a), rho * grow * math.sin(a), z) for rho, z in sections]
        sec_b = [(rho * grow * math.cos(b), rho * grow * math.sin(b), z) for rho, z in sections]
        out.append((tuple(sec_a), tuple(sec_b), color, visual) if visual else (tuple(sec_a), tuple(sec_b), color))
    return out


def _cup_shapes():
    """(hulls, spheres, cylinders) of one cup in its own frame."""
    z1 = BEAD_ZC
    wall = _ring_prisms([(R0, 0.0), (wall_radius(z1), z1), (wall_radius(z1) - WALL_T, z1), (R0 - WALL_T, 0.0)],
                        WALL_SEGS, PAPER)
    hexagon = [(BEAD_RC + BEAD_R * math.cos(t), BEAD_ZC + BEAD_R * math.sin(t))
               for t in (math.radians(d) for d in (0, 60, 120, 180, 240, 300))]
    bead = _ring_prisms(hexagon, BEAD_SEGS, PAPER)
    # four stacking bumps inside, just above the bottom: the next cup's bottom edge rests on their tops
    lug_r = R0 - WALL_T / 2
    lugs = [((lug_r * math.cos(t), lug_r * math.sin(t), NEST_PITCH - LUG_R), LUG_R, PAPER)
            for t in (math.radians(45 + 90 * i) for i in range(4))]
    base_r = wall_radius(BASE_Z[0]) - WALL_T - 0.0003
    base = [((0.0, 0.0, sum(BASE_Z) / 2), base_r, BASE_Z[1] - BASE_Z[0], PAPER)]
    return wall + bead, lugs, base


# -- the dispenser: a stainless tube hung on the wall, its gasket bezel resting on a lip at the tube's bottom ----------
AXIS_XY = (0.38, 0.14)  # the tube's axis on the counter (world x, y): clear of the left arm's ready pose (0.32, 0.305)
# The gasket bezel's bottom face (the lip's top), 24.5 cm above the counter. A pull-down dispenser is mounted a full
# cup height plus a little above the counter: the bottom cup only comes free once it has slid down off the next cup,
# which by then hangs where it hung (seen on Brev with the outlet at 15 cm: the pulled cup was still round the next).
OUTLET_Z = TOP + 0.2452
TUBE_IN = 0.0535  # tube inside radius
TUBE_T = 0.0015
TUBE_OUT = TUBE_IN + TUBE_T
TUBE_L = 0.41  # 16 in, from the bezel's bottom face to the cap
LIP_IN = 0.047  # the tube's bottom lip, under the bezel: a cup's rim (45 mm) passes inside it
LIP_T = 0.003
TUBE_SEGS = 24
BRACKET_GAP = 0.012  # tube to wall
WALL_X = AXIS_XY[0] + TUBE_OUT + BRACKET_GAP  # the wall's face (world x)
WALL_SIZE = (0.02, 0.80, 0.72)
WALL_Y = 0.15  # the wall's middle (world y)


def _dispenser():
    """Static: the tube, its bottom lip, its cap, two brackets and the wall it hangs on (part frame: origin on the
    tube's axis at the lip's top face)."""
    tube = _ring_prisms([(TUBE_IN, -LIP_T), (TUBE_OUT, -LIP_T), (TUBE_OUT, TUBE_L), (TUBE_IN, TUBE_L)], TUBE_SEGS,
                        STEEL)
    lip = _ring_prisms([(LIP_IN, -LIP_T), (TUBE_IN, -LIP_T), (TUBE_IN, 0.0), (LIP_IN, 0.0)], TUBE_SEGS, STEEL)
    cap = [((0.0, 0.0, TUBE_L + 0.006), TUBE_OUT + 0.002, 0.012, STEEL)]
    bx = TUBE_OUT + BRACKET_GAP / 2  # from the tube's back face (a flat facet at +x) to the wall, 1 mm into each
    brackets = [((bx, 0.0, z), (BRACKET_GAP + 0.002, 0.05, 0.03), STEEL) for z in (0.06, TUBE_L - 0.06)]
    wall_z = (TOP - OUTLET_Z) + WALL_SIZE[2] / 2
    wall = [((WALL_X - AXIS_XY[0] + WALL_SIZE[0] / 2, WALL_Y - AXIS_XY[1], wall_z), WALL_SIZE, WALL_WHITE)]
    return part("dispenser", pos=(AXIS_XY[0], AXIS_XY[1], OUTLET_Z), mass=None, color=STEEL, friction=0.4,
                boxes=brackets + wall, cylinders=cap, hulls=tube + lip)


# the gasket: a black silicone baffle in a bezel ring that rests on the lip, inside the tube. Its eight flaps are
# sprung fingers with rounded tips; each gives outward and downward (45 degrees) when a rim is pulled down through it.
BEZEL_IN = 0.048  # the ring's inside radius: a centred rim (45 mm) clears it by 3 mm
BEZEL_OUT = TUBE_IN - 0.0005
BEZEL_H = 0.016
BEZEL_MASS = 0.05
FLAPS = 8
FLAP_TIP_R = 0.003  # each flap's rounded tip
FLAP_Z = 0.011  # the tips' centres, above the bezel's bottom face
APERTURE = 0.0425  # the flaps' tips reach in to this radius: 2.5 mm inside the rim's outside, so they hold it
SLIDE_DOWN = math.radians(45.0)  # a flap gives along this angle below horizontal, outward
FLAP_TRAVEL = 0.0065  # along its slide: 4.6 mm outward, past the rim's 2.5 mm
FLAP_RATE = 60.0  # N/m along the slide, each
FLAP_DAMPING = 0.4
FLAP_MASS = 0.002
FLAP_FRICTION = 0.7  # silicone
FLAP_RC = APERTURE + FLAP_TIP_R  # the tips' centres from the axis at rest


def flap_angle(i: int) -> float:
    """Every 45 degrees from +x: each flap meets the middle of one of the cup wall's flat facets (every 15 degrees)."""
    return 2 * math.pi * i / FLAPS


def flap_home(i: int) -> np.ndarray:
    """A flap tip's centre at rest, in the bezel's frame."""
    t = flap_angle(i)
    return np.array([FLAP_RC * math.cos(t), FLAP_RC * math.sin(t), FLAP_Z])


def flap_direction(i: int) -> np.ndarray:
    """The flap's slide axis (+ is back toward rest: inward and up); it gives along minus this."""
    t = flap_angle(i)
    return np.array([-math.cos(t) * math.cos(SLIDE_DOWN), -math.sin(t) * math.cos(SLIDE_DOWN), math.sin(SLIDE_DOWN)])


def _flap(i: int) -> Slider:
    home = flap_home(i)
    t = flap_angle(i)
    w = math.radians(7.0)
    # a visual-only web from the tip back to the bezel ring, so the baffle reads as a gasket (no collider: the flap's
    # contact is its rounded tip)
    web = [(FLAP_RC, FLAP_Z - 0.0012), (BEZEL_IN + 0.001, FLAP_Z - 0.0012), (BEZEL_IN + 0.001, FLAP_Z + 0.0012),
           (FLAP_RC, FLAP_Z + 0.0012)]
    sec_a = tuple((r * math.cos(t - w), r * math.sin(t - w), z) for r, z in web)
    sec_b = tuple((r * math.cos(t + w), r * math.sin(t + w), z) for r, z in web)
    return Slider(f"flap{i}", rest=tuple(float(v) for v in home), boxes=(),
                  spheres=((tuple(float(v) for v in home), FLAP_TIP_R, SILICONE),),
                  hulls=((sec_a, sec_b, SILICONE, True),), travel=FLAP_TRAVEL, rate=FLAP_RATE, damping=FLAP_DAMPING,
                  mass=FLAP_MASS, friction=FLAP_FRICTION, direction=tuple(float(v) for v in flap_direction(i)))


def _bezel():
    ring = _ring_prisms([(BEZEL_IN, 0.0), (BEZEL_OUT, 0.0), (BEZEL_OUT, BEZEL_H), (BEZEL_IN, BEZEL_H)], TUBE_SEGS,
                        SILICONE)
    return part("gasket", pos=(AXIS_XY[0], AXIS_XY[1], OUTLET_Z + 0.0003), mass=BEZEL_MASS, color=SILICONE,
                friction=FLAP_FRICTION, boxes=(), hulls=ring, sliders=tuple(_flap(i) for i in range(FLAPS)))


# where the bottom cup hangs: its rim's roll resting on the flap tips (centres 4.6 mm apart, the roll 2.1 mm further in)
_REST_DR = FLAP_RC - BEAD_RC
_REST_DZ = math.sqrt((FLAP_TIP_R + BEAD_R) ** 2 - _REST_DR ** 2)
CUP1_BASE_Z = OUTLET_Z + FLAP_Z + _REST_DZ - BEAD_ZC  # world z of the bottom cup's bottom edge (spawn)
REST_SINK = 0.0019  # measured on Brev (2026-11-15): under the stack's weight the flaps give 1.4 mm and the cup settles 1.9 mm
CUP1_HANG_Z = CUP1_BASE_Z - REST_SINK  # where the bottom cup's bottom edge hangs at rest (world z)


def cup_name(k: int) -> str:
    return f"cup{k}"


def _cup(k: int):
    hulls, lugs, base = _cup_shapes()
    z = CUP1_BASE_Z + (k - 1) * NEST_PITCH + 0.0004
    return part(cup_name(k), pos=(AXIS_XY[0], AXIS_XY[1], z), mass=CUP_MASS, color=PAPER, friction=CUP_FRICTION,
                boxes=(), hulls=hulls, spheres=lugs, cylinders=base)


DISPENSER = _dispenser()
GASKET = _bezel()
CUPS = [_cup(k) for k in range(1, N_CUPS + 1)]

# a black rubber bar mat on the counter, right of the dispenser
MAT_XY = (0.34, -0.03)
MAT_SIZE = (0.16, 0.12, 0.004)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("mat", pos=(MAT_XY[0], MAT_XY[1], TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=RUBBER)

OBJECTS = [DISPENSER, GASKET, *CUPS, MAT]

# -- scoring -------------------------------------------------------------------------------------------------------------
DISPENSER_BOTTOM = OUTLET_Z - LIP_T  # the dispenser's lowest edge (world z)
GASKET_Z = OUTLET_Z + FLAP_Z  # the flaps' plane
UPRIGHT_TOL = 10.0  # degrees: standing
TIPPED = 60.0  # degrees: a cup that ever tips this far has been dropped or knocked over (it would be thrown away)
ON_SURFACE = 0.006  # the cup's bottom edge within this of the counter or the mat
LET_GO = 0.040  # each TCP at least this far from the cup: the finger surfaces reach about 2.4 cm from the TCP, so
#                 no part of a gripper is within about 1.5 cm of it
OFF_TABLE = 0.05  # below the counter by this: on the floor
HELD_DROP = 0.005  # a cup still in the dispenser hangs no more than this below where the bottom cup hung at the start
# Crushing. The sim's cups are rigid, so a crushed cup is scored from the grip force: a gripper squeezing a cup harder
# than CRUSH_N has dented it (it would be thrown away). For scale, an ordinary paper cup's wall gives about 6 mm under
# 5 N at two-thirds of its height; near the bottom, where it is pinched here, the bottom disc stiffens it. The YAM's
# gripper squeezes up to 50 N when told to close fully. The force is read from the fingers' position error times the
# gripper's stiffness (yam_arena's implicit gripper actuator: 5,000 N/m, capped at 50 N), taking the smaller of the two
# fingers (both press on a held cup; a finger held back by anything else would read high on its own).
CRUSH_N = 20.0
NEAR_CUP = 0.01  # a gripper counts as holding a cup when its TCP is within this of the cup's outside


def _fresh() -> dict:
    return {"tipped": False, "fell": False, "crushed": False, "max_squeeze": 0.0, "squeeze": [], "tcps": [],
            "steps": 0, "squeeze_error": None}


_S = _fresh()
_DEBUG = bool(os.environ.get("CUP_DEBUG"))


def tilt_deg(state) -> float:
    z = state.rotation()[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def rim_top(state) -> float:
    """World z of the top of the cup's rim (its highest point when upright)."""
    return float(state.to_world((0.0, 0.0, CUP_H))[2])


def below_dispenser(state) -> bool:
    """The whole cup, rim included, below the dispenser's bottom edge."""
    top = max(float(state.to_world((dx, dy, CUP_H))[2]) for dx, dy in ((0, 0), (RIM_D / 2, 0), (-RIM_D / 2, 0),
                                                                          (0, RIM_D / 2), (0, -RIM_D / 2)))
    return top < DISPENSER_BOTTOM


def nested(a, b) -> bool:
    """Cups ``a`` and ``b`` still one inside the other: their axes within a rim's radius of each other and their spans
    (bottom to rim) overlapping in height."""
    if math.hypot(float(a.pos[0] - b.pos[0]), float(a.pos[1] - b.pos[1])) > RIM_D / 2:
        return False
    return float(a.pos[2]) < rim_top(b) and float(b.pos[2]) < rim_top(a)


def is_out(state, others=()) -> bool:
    """Out of the dispenser: the whole cup below the dispenser's bottom edge and clear of every other cup (a cup pulled
    only partway still has the next cup's bottom nested in it)."""
    return below_dispenser(state) and not any(nested(state, o) for o in others)


def is_held(state) -> bool:
    """Still in the dispenser: hanging upright, no more than HELD_DROP below where the bottom cup hung at the start (a
    cup dragged partly through the gasket hangs lower; one higher up is nested inside the bottom one)."""
    return (float(state.pos[2]) >= CUP1_HANG_Z - HELD_DROP and tilt_deg(state) < UPRIGHT_TOL
            and not below_dispenser(state))


def cup_distance(state, point) -> float:
    """How far ``point`` (world) is from the cup's outside (a cylinder as wide as the rim, base to rim; 0 inside)."""
    local = state.to_local(np.asarray(point, float))
    radial = max(0.0, math.hypot(local[0], local[1]) - RIM_D / 2)
    axial = max(0.0, -local[2], local[2] - CUP_H)
    return math.hypot(radial, axial)


def released(state) -> bool:
    return bool(_S["tcps"]) and all(cup_distance(state, t) >= LET_GO for t in _S["tcps"])


def standing_on(state) -> str | None:
    """'mat' or 'counter' if the cup stands upright with its bottom edge on that surface, else None."""
    if tilt_deg(state) > UPRIGHT_TOL:
        return None
    bottom = float(state.pos[2])
    x, y = float(state.pos[0]), float(state.pos[1])
    on_mat_xy = abs(x - MAT_XY[0]) <= MAT_SIZE[0] / 2 and abs(y - MAT_XY[1]) <= MAT_SIZE[1] / 2
    if on_mat_xy and abs(bottom - MAT_TOP) <= ON_SURFACE:
        return "mat"
    if abs(bottom - TOP) <= ON_SURFACE:
        return "counter"
    return None


def wholly_on_mat(state) -> bool:
    """The cup's whole bottom on the mat: its centre at least the bottom's radius inside every edge of the mat."""
    x, y = float(state.pos[0]), float(state.pos[1])
    return (abs(x - MAT_XY[0]) <= MAT_SIZE[0] / 2 - R0 and abs(y - MAT_XY[1]) <= MAT_SIZE[1] / 2 - R0
            and standing_on(state) == "mat")


def score(objects) -> float:
    cups = [objects[c.name] for c in CUPS]
    if _S["tipped"] or _S["fell"] or _S["crushed"]:
        return 0.0
    below = [c for c in cups if below_dispenser(c)]
    if len(below) != 1:  # none pulled, or two or more pulled out
        return 0.0
    cup = below[0]
    if not is_out(cup, [c for c in cups if c is not cup]):  # pulled only partway: still round the next cup
        return 0.0
    others_held = all(is_held(c) for c in cups if c is not cup)
    if not others_held:
        return 0.25
    if standing_on(cup) is None or not released(cup) or not cup.resting():
        return 0.25
    return 1.0 if wholly_on_mat(cup) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0


def grippers(environment) -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips)."""
    from yam_arena.runtime import tcp_world

    base = environment.unwrapped
    origin = _np_vec(base.scene.env_origins[0])
    return [tcp_world(base.scene[key]) - origin for key in ("left_robot", "right_robot")]


def _np_vec(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def squeeze_force(finger_q, grip: float) -> float:
    """The force (N) a gripper squeezes with: its finger joints at ``finger_q`` (m) and its normalized grip command
    ``grip`` (0 closed .. 1 open; the fingers' target is grip x FINGER_OPEN). A finger short of its target pushes with
    stiffness x shortfall, capped at the effort limit; the smaller of the two is what the held object feels."""
    target = float(grip) * FINGER_OPEN
    forces = [min(GRIPPER["effort"], max(0.0, GRIPPER["stiffness"] * (target - float(q)))) for q in finger_q]
    return min(forces)


def squeezes(environment) -> list[float]:
    """Each arm's squeeze force (N), from its finger joints and its gripper action term's command."""
    base = environment.unwrapped
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        ids = robot.find_joints(list(FINGER_JOINTS), preserve_order=True)[0]
        q = _np_vec(robot.data.joint_pos)[0, ids]
        grip = _np_vec(base.action_manager.get_term(f"{side}_gripper").processed_actions)[0, 0]
        out.append(squeeze_force(q, grip))
    return out


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(_fresh())


@on_step
def _follow(environment) -> None:
    _S["steps"] += 1
    states = {c.name: _part_state(environment, _SPECS[c.name]) for c in CUPS}
    for st in states.values():
        if tilt_deg(st) > TIPPED:
            _S["tipped"] = True
        if float(st.pos[2]) < TOP - OFF_TABLE:
            _S["fell"] = True
    try:
        _S["tcps"] = grippers(environment)
    except (KeyError, ValueError, AttributeError):
        _S["tcps"] = []
    try:
        _S["squeeze"] = squeezes(environment)
    except Exception as error:  # noqa: BLE001 - report once; GO_LIVE.md says to check for this line
        _S["squeeze"] = []
        if _S["squeeze_error"] is None:
            _S["squeeze_error"] = repr(error)
            print(f"[cup] WARNING: can't read the grip force, crushing is not scored: {error!r}", flush=True)
    for force, tcp in zip(_S["squeeze"], _S["tcps"]):
        if any(cup_distance(st, tcp) <= NEAR_CUP for st in states.values()):
            _S["max_squeeze"] = max(_S["max_squeeze"], force)
            if force > CRUSH_N:
                _S["crushed"] = True
    if _DEBUG and _S["steps"] % 5 == 0:
        gasket = _part_state(environment, _SPECS[GASKET.name])
        flaps = [round(v * 1000, 2) for v in gasket.sliders.values()]
        parts = [f"{n}: base {np.round(s.pos, 4).tolist()} tilt {tilt_deg(s):.1f} rim {rim_top(s):.4f} "
                 f"out {is_out(s, [o for m, o in states.items() if m != n])} held {is_held(s)}"
                 for n, s in states.items()]
        print(f"[cup] step {_S['steps']} tipped {_S['tipped']} fell {_S['fell']} crushed {_S['crushed']} "
              f"squeeze(N) {[round(f, 1) for f in _S['squeeze']]} max {_S['max_squeeze']:.1f} "
              f"tcps {[np.round(t, 4).tolist() for t in _S['tcps']]} gasket {np.round(gasket.pos, 4).tolist()} "
              f"flaps(mm) {flaps} | " + " | ".join(parts), flush=True)


# -- a tilted grip for the scripted demo ---------------------------------------------------------------------------------
# yam_arena's waypoints carry only a yaw about the vertical (a straight-down grip). The cup hangs under the dispenser, so
# the demo grips it with the approach tipped forward: a Tilt is a yaw that also carries that pitch, and the hook below
# teaches yam_arena.demo's grip_rotation to apply it. Plain floats behave as before.


class Tilt(float):
    """A grip yaw (rad) that also carries a pitch (rad): the approach axis tipped from straight down toward the yawed
    +x by ``pitch``."""

    pitch: float

    def __new__(cls, yaw: float = 0.0, pitch: float = 0.0):
        obj = super().__new__(cls, yaw)
        obj.pitch = float(pitch)
        return obj

    def __repr__(self) -> str:
        return f"Tilt(yaw={float(self):.4f}, pitch={self.pitch:.4f})"


def _rz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


if not hasattr(_demo, "_untilted_grip_rotation"):
    _demo._untilted_grip_rotation = _demo.grip_rotation


def tilted_grip_rotation(yaw: float = 0.0) -> np.ndarray:
    down = _demo._untilted_grip_rotation(0.0)
    pitch = getattr(yaw, "pitch", 0.0)
    if not pitch:
        return _demo._untilted_grip_rotation(float(yaw))
    return _rz(float(yaw)) @ _ry(-pitch) @ down


_demo.grip_rotation = tilted_grip_rotation

# -- the scripted demo ---------------------------------------------------------------------------------------------------
# The left arm pinches the hanging cup across its sides (fingers closing along world y) with the approach tipped 62
# degrees forward, so the gripper stays clear below and in front of the dispenser; pulls it straight down until the rim
# snaps through the flaps and the whole cup is below the dispenser; brings it over the mat; sets it down and lets go.
# Pitch, pinch height and pinch offset were picked offline with the YAM's meshes (check_clearance.py): the closest
# approaches are 5.8 mm to the dispenser, 4.8 mm to the cup on the way in, 11.6 mm to the counter.
# Straight down is impossible (the gripper would be inside the dispenser), and from 70 degrees the arm can't reach the
# set-down pose.

PITCH = math.radians(float(os.environ.get("CUP_PITCH_DEG", "62")))
GRIP = Tilt(0.0, PITCH)
GRIP_UP = float(os.environ.get("CUP_GRIP_UP", "0.019"))  # pinch this far above the cup's bottom edge
R_GRIP = wall_radius(GRIP_UP)
# The tilted pads (27.4 mm wide) span 12 mm up and down the cup's side from the pinch, and the cup widens upward, so they
# stop on it at the radius their upper edge meets (seen on Brev: 33.6 mm, not 32.3).
PAD_HALF_W = 0.0137
R_CONTACT = wall_radius(GRIP_UP + PAD_HALF_W * math.sin(PITCH))
# The fingers' roots join across the gap 44 mm behind the TCP, so a tilted pinch on an upright cup takes the cup near the
# pad tips: the TCP sits this far behind the cup's axis along the approach, keeping the cup's near side out of the gap.
GRIP_FORWARD = float(os.environ.get("CUP_GRIP_FORWARD", "0.014"))
# Both fingers close and centre the cup between them (seen on Brev, 2026-11-15: pinched off-centre, the hanging cup
# was pulled across to the TCP and squeezed at the 50 N cap). So the TCP aims at the cup's axis, and a paper cup is
# pinched gently: the fingers' target is just inside the cup's sides, so each pad stops on the cup R_GRIP from the TCP
# and pushes with about GRIP_SQUEEZE_N (well under CRUSH_N; closing fully would squeeze 50 N). A finger joint's value is
# minus its pad's distance from the TCP (0 closed, -47 mm open).
GRIP_SQUEEZE_N = float(os.environ.get("CUP_SQUEEZE_N", "6"))
GRIP_FINGER = -R_CONTACT + GRIP_SQUEEZE_N / GRIPPER["stiffness"]
# The cup comes down a cup height plus a little: its rim through the flaps (after about 9 mm, at about 2 N), the next
# cup dropping a pitch onto the flaps with it, then sliding on down off the next cup until its rim is below that cup's
# bottom. The arm aims PULL_OVER further: it lags against the flaps (~500 N/m down) until the rim lets go.
PULL = CUP_H + 0.006
PULL_OVER = 0.006
BACK = 0.05  # out from under the dispenser, toward the arms
SET_OVER = 0.002  # the set-down aims the cup's bottom this far into the mat, so it seats
RETREAT = 0.06
SAG = tuple(float(v) for v in os.environ.get("CUP_SAG", "0,0,0").split(","))  # tune on Brev: "dx,dy,dz"


def approach_dir() -> np.ndarray:
    return np.array([math.sin(PITCH), 0.0, -math.cos(PITCH)])


def tcp_for_cup(cup_xy, bottom_z) -> tuple[float, float, float]:
    """The TCP aim that pinches a cup standing with its axis at ``cup_xy`` and its bottom edge at ``bottom_z``: the pads
    on its sides GRIP_UP above its bottom edge, the TCP GRIP_FORWARD back along the approach from its axis."""
    a = approach_dir()
    x = cup_xy[0] - GRIP_FORWARD * a[0]
    z = bottom_z + GRIP_UP - GRIP_FORWARD * a[2]
    return (float(x + SAG[0]), float(cup_xy[1] + SAG[1]), float(z + SAG[2]))


PARK = {"left": (0.18, 0.34, TOP + 0.16), "right": (0.18, -0.34, TOP + 0.16)}
COUNTER_SPOT = (0.34, 0.27)  # the 'counter' shortcut stands the cup here: on the counter left of the dispenser, off the mat


def _take(place: str = "mat", release: bool = True) -> list[Waypoint]:
    """Pinch the hanging cup, pull it out, and stand it on the mat (``place='mat'``), on the counter left of the
    dispenser (``'counter'``) or keep holding it in the air (``'air'``)."""
    g = GRIP
    pinch = np.array(tcp_for_cup(AXIS_XY, CUP1_HANG_Z))
    start = pinch + np.array([-0.08, 0.0, 0.0])
    pulled = pinch + np.array([0.0, 0.0, -(PULL + PULL_OVER)])
    out = pulled + np.array([-BACK, 0.0, 0.0])
    wps = [
        Waypoint(2.0, tuple(start + np.array([0, 0, 0.03])), FINGER_OPEN, "in front of the cup, tilted", g),
        Waypoint(1.2, tuple(start), FINGER_OPEN, "down to the pinch height", g),
        Waypoint(1.8, tuple(pinch), FINGER_OPEN, "fingers either side of the cup", g),
        Waypoint(0.6, tuple(pinch), FINGER_OPEN, "steady before the pinch", g),
        Waypoint(0.8, tuple(pinch), GRIP_FINGER, "pinch", g),
        Waypoint(4.0, tuple(pulled), GRIP_FINGER, "pull straight down through the gasket", g),
        Waypoint(0.6, tuple(pulled), GRIP_FINGER, "steady", g),
        Waypoint(1.5, tuple(out), GRIP_FINGER, "out from under the dispenser", g),
    ]
    if place == "air":
        return wps + [Waypoint(60.0, tuple(out), GRIP_FINGER, "keep holding it", g)]
    spot = MAT_XY if place == "mat" else COUNTER_SPOT
    floor = MAT_TOP if place == "mat" else TOP
    above = np.array(tcp_for_cup(spot, floor + 0.02))
    down = np.array(tcp_for_cup(spot, floor - SET_OVER))
    back = down - RETREAT * approach_dir()
    wps += [
        Waypoint(2.2, tuple(above), GRIP_FINGER, f"over the {place}", g),
        Waypoint(1.2, tuple(down), GRIP_FINGER, f"set it down on the {place}", g),
        Waypoint(0.5, tuple(down), GRIP_FINGER, "settle", g),
    ]
    if not release:
        return wps + [Waypoint(60.0, tuple(down), GRIP_FINGER, "keep holding it", g)]
    return wps + [
        Waypoint(0.8, tuple(down), FINGER_OPEN, "let go", g),
        Waypoint(0.5, tuple(down), FINGER_OPEN, "fingers open", g),
        Waypoint(1.2, tuple(back), FINGER_OPEN, "back away", g),
        Waypoint(1.5, PARK["left"], FINGER_OPEN, "park"),
    ]


def _wait(duration: float, pos, finger=FINGER_OPEN, label="wait") -> list[Waypoint]:
    return [Waypoint(duration, pos, finger, label)]


def demo():
    """DEMO_VARIANT: full (1.0), nothing (0), pull_only (hold it in the air: 0.25), counter (stand it on the counter
    left of the dispenser, off the mat: 0.5), keep_holding (on the mat but never let go: 0.25), probe (pinch and pull
    only)."""
    variant = os.environ.get("DEMO_VARIANT", "full")
    right = _wait(3.0, PARK["right"])
    if variant == "nothing":
        return {"left": _wait(3.0, PARK["left"]), "right": right}
    if variant in ("pull_only", "probe"):
        return {"left": _take("air"), "right": right}
    if variant == "counter":
        return {"left": _take("counter"), "right": right}
    if variant == "keep_holding":
        return {"left": _take("mat", release=False), "right": right}
    return {"left": _take("mat"), "right": right}

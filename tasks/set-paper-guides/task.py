"""On the station table: a grey A4 printer paper cassette (outside 372 x 272 x 36 mm, inside 360 x 260 mm) lies with
its long axis pointing away from the arms. A stack of 100 sheets of A4 paper (297 x 210 x 10 mm, 0.5 kg) lies in it,
in the far-left corner: the corner the printer feeds from. The cassette has two orange paper guides, both wide open:
a side guide on the right (it slides left, toward the stack's right edge) and a rear guide at the near end (it slides
forward, toward the stack's near edge). Each guide is locked in place unless its orange release tab (30 mm wide, on
top of the guide) is squeezed from both sides; while it is squeezed the guide slides freely, and it locks again where
it is let go. The robot must set both guides snug against the stack, so the sheets cannot skew when they feed, without
moving the stack, and let go of both tabs.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import PartState, Slider, finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Set the paper tray's two guides snug against the paper stack, then let go of them."

GREY = (0.62, 0.63, 0.65)
ORANGE = (1.00, 0.50, 0.00)
WHITE = (0.96, 0.96, 0.94)


def _mm(x0, x1, y0, y1, z0, z1, color):
    """A box spec from its extents in millimetres (part frame)."""
    return (((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000),
            ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000), color)


# -- the cassette: origin on the table under its centre; inside x +-180 mm, y +-130 mm; floor top z 4 mm --------------
FLOOR_Z = 0.004
TRAY_BOXES = [
    _mm(-186, 186, -136, 136, 0, 4, GREY),        # floor
    _mm(-186, 186, 130, 136, 4, 36, GREY),        # left wall (+y): the stack's datum side
    _mm(-186, 186, -136, -130, 4, 36, GREY),      # right wall (-y), behind the side guide
    _mm(180, 186, -130, 130, 4, 36, GREY),        # far wall (+x): the feed edge
    _mm(-186, -180, -130, 130, 4, 36, GREY),      # near wall
]

# -- the stack: rigid, spawned 1.5 mm off the far and left walls and 1 mm above the floor ------------------------------
STACK_SIZE = (0.297, 0.210, 0.010)
STACK_LOCAL = (0.180 - 0.0015 - STACK_SIZE[0] / 2, 0.130 - 0.0015 - STACK_SIZE[1] / 2)   # (30.0, 23.5) mm

# -- the guides. Each: a 6 mm thick fence (z 5-39 mm, 1 mm clear of the floor) and a 20 x 30 x 30 mm release tab on
# top (z 39-69 mm), 30 mm wide along the guide's slide axis, so the fingers squeeze it along that axis. -----------------
GUIDE_HALF_T = 0.003
TAB_Z0, TAB_Z1 = 0.039, 0.069
TAB_HALF_ALONG, TAB_HALF_ACROSS = 0.015, 0.010
LOCK_RATE = 20000.0     # N/m while locked: the arm can push ~10 N, so a locked guide gives about 0.5 mm


class Guide:
    """One paper guide: slides along ``axis`` (0 = x, 1 = y, part frame) in the + direction from ``open_at``."""

    def __init__(self, name, axis, open_at, along_tab, fence_span, travel):
        self.name, self.axis, self.open_at, self.travel = name, axis, open_at, travel
        self.tab_c = along_tab            # tab centre on the other horizontal axis
        lo, hi = fence_span
        a0, a1 = 1000 * (open_at - GUIDE_HALF_T), 1000 * (open_at + GUIDE_HALF_T)
        t0, t1 = 1000 * (open_at - TAB_HALF_ALONG), 1000 * (open_at + TAB_HALF_ALONG)
        c0, c1 = 1000 * (along_tab - TAB_HALF_ACROSS), 1000 * (along_tab + TAB_HALF_ACROSS)
        if axis == 1:
            boxes = (_mm(1000 * lo, 1000 * hi, a0, a1, 5, 39, ORANGE), _mm(c0, c1, t0, t1, 39, 69, ORANGE))
            rest = (along_tab, open_at, 0.022)
        else:
            boxes = (_mm(a0, a1, 1000 * lo, 1000 * hi, 5, 39, ORANGE), _mm(t0, t1, c0, c1, 39, 69, ORANGE))
            rest = (open_at, along_tab, 0.022)
        self.slider = Slider(name, rest=rest, travel=travel, rate=LOCK_RATE, damping=40.0, mass=0.03,
                             axis="XY"[axis], sign=1, boxes=boxes)

    def tab_centre(self, q):
        """The tab's centre (part frame, at mid-height) with the guide slid by q."""
        p = [0.0, 0.0, (TAB_Z0 + TAB_Z1) / 2]
        p[self.axis], p[1 - self.axis] = self.open_at + q, self.tab_c
        return np.array(p)

    def face(self, q):
        """Where the guide's stack-side face is along its axis (part frame)."""
        return self.open_at + GUIDE_HALF_T + q

    def stack_edge(self, s):
        """The stack edge this guide closes on, along its axis, for a stack centred at s (part frame)."""
        return s[self.axis] - STACK_SIZE[self.axis] / 2


SIDE = Guide("side_guide", axis=1, open_at=-0.120, along_tab=STACK_LOCAL[0], fence_span=(-0.070, 0.130),
             travel=0.060)
REAR = Guide("rear_guide", axis=0, open_at=-0.170, along_tab=STACK_LOCAL[1], fence_span=(-0.060, 0.110),
             travel=0.070)
GUIDES = (SIDE, REAR)

TRAY_XY = (0.31, 0.02)
TRAY = part("tray", pos=(*TRAY_XY, TOP), mass=5.0, boxes=TRAY_BOXES, sliders=(SIDE.slider, REAR.slider), color=GREY,
            friction=0.8)
STACK = part("stack", pos=(TRAY_XY[0] + STACK_LOCAL[0], TRAY_XY[1] + STACK_LOCAL[1], TOP + FLOOR_Z + 0.001),
             mass=0.5, boxes=[((0, 0, STACK_SIZE[2] / 2), STACK_SIZE, WHITE)], color=WHITE, friction=0.8)
OBJECTS = [TRAY, STACK]

# -- the release-tab lock ----------------------------------------------------------------------------------------------
# finger_points() gives, per arm, points 6 mm outside each finger pad, at the TCP and at the pad tips. With the pads on
# a 30 mm tab they are 42 mm apart; dragging a guide spreads the fingers to ~52 mm; fully open is ~106 mm.
SQUEEZE_GAP = 0.060
TOUCH = 0.020           # a pad point this close to a tab (or anywhere over the cassette's guides) counts as touching
_S: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _S.update(lock={g.name: 0.0 for g in GUIDES}, squeezed={g.name: False for g in GUIDES},
              stiff={g.name: None for g in GUIDES}, touching=False, finger_check=None,
              tray0=None, stack0=None, failed=False, why="", steps=0, last=None)


_reset()


def squeezed_by(guide: Guide, a_local, b_local, q) -> bool:
    """Two pad points of one arm (part frame) squeeze ``guide``'s tab: one on each side of it along the slide axis,
    both level with it and over its width, at most SQUEEZE_GAP apart."""
    a, b = np.asarray(a_local, float), np.asarray(b_local, float)
    c = guide.tab_centre(q)
    k, o = guide.axis, 1 - guide.axis
    lo, hi = sorted((a[k], b[k]))
    straddle = lo <= c[k] - TAB_HALF_ALONG + 0.004 and hi >= c[k] + TAB_HALF_ALONG - 0.004
    level = all(abs(p[o] - c[o]) <= TAB_HALF_ACROSS + 0.008 and TAB_Z0 - 0.002 <= p[2] <= TAB_Z1 + 0.006
                for p in (a, b))
    return bool(straddle and level and float(np.linalg.norm(a - b)) <= SQUEEZE_GAP)


def _near_tab(guide: Guide, p, q) -> bool:
    d = np.abs(np.asarray(p) - guide.tab_centre(q))
    half = np.array([0.0, 0.0, (TAB_Z1 - TAB_Z0) / 2])
    half[guide.axis], half[1 - guide.axis] = TAB_HALF_ALONG, TAB_HALF_ACROSS
    return bool(np.linalg.norm(np.maximum(d - half, 0.0)) < TOUCH)


def _drive(environment, guide: Guide, q, stiffness) -> None:
    """Point a guide's drive at q; set its stiffness when it changes (0 = free while squeezed, LOCK_RATE = locked)."""
    tray = environment.unwrapped.scene[TRAY.name]
    j = list(tray.joint_names).index(f"{guide.name}_slide")
    if _S["stiff"][guide.name] != stiffness:
        tray.write_joint_stiffness_to_sim(float(stiffness), joint_ids=[j])
        _S["stiff"][guide.name] = stiffness
    target = tray.data.joint_pos_target.clone()
    target[:, j] = float(q)
    tray.set_joint_position_target(target)


@on_step
def _lock(environment) -> None:
    """Unlock a guide while a gripper squeezes its tab; otherwise hold it where it was last let go. Also record
    whether any finger pad is touching a tab (score() cannot see the robot)."""
    from yam_arena.scene import object_states

    tray = object_states(environment, [TRAY.name])[TRAY.name]
    try:
        pts = [tray.to_local(p) for p in finger_points(environment)]
        _S["finger_check"] = True
    except Exception as exc:  # noqa: BLE001 - report once; an unknown grip never unlocks and never counts as released
        if _S["finger_check"] is None:
            print(f"set-paper-guides: finger check unavailable: {exc!r}", flush=True)
        _S["finger_check"], pts = False, []
    touching = not pts and _S["finger_check"] is False
    for g in GUIDES:
        q = tray.sliders[g.name]
        sq = any(squeezed_by(g, pts[k], pts[k + 1], q) for k in range(0, len(pts), 2))
        if sq:
            _S["lock"][g.name] = q
        _S["squeezed"][g.name] = sq
        _drive(environment, g, _S["lock"][g.name], 0.0 if sq else LOCK_RATE)
        touching |= any(_near_tab(g, p, q) for p in pts)
    _S["touching"] = touching
    _S["steps"] += 1
    if os.environ.get("TASK_DEBUG") and _S["steps"] % 20 == 0:
        print(f"step {_S['steps']}: {_S['last']} squeezed {_S['squeezed']} touching {touching}", flush=True)


# -- scoring -----------------------------------------------------------------------------------------------------------
SNUG, NEAR = 0.001, 0.005           # guide-to-stack gap for full and half credit per guide
STACK_MOVE, STACK_TURN, STACK_LIFT = 0.005, 3.0, 0.010
TRAY_MOVE = 0.005


def _yaw(p: PartState) -> float:
    r = p.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def measure(objects) -> dict:
    """Stack centre in the cassette frame, each guide's gap to its stack edge (m), and the stack's yaw in the
    cassette (deg)."""
    tray, stack = objects[TRAY.name], objects[STACK.name]
    s = tray.to_local(stack.pos)
    gaps = {g.name: float(g.stack_edge(s) - g.face(tray.sliders[g.name])) for g in GUIDES}
    turn = (_yaw(stack) - _yaw(tray) + 180) % 360 - 180
    return {"stack": s, "gaps": gaps, "turn": turn}


def score(objects) -> float:
    """Per guide: 0.5 if it ends locked with at most 1 mm between it and the stack's edge, 0.25 if at most 5 mm.
    0 if at any time the stack moved 5 mm, turned 3 degrees or was lifted 10 mm, or the cassette moved 5 mm; and 0
    while a finger pad is within 2 cm of a tab (both arms must have let go)."""
    tray, stack = objects[TRAY.name], objects[STACK.name]
    m = measure(objects)
    if _S["stack0"] is None:
        _S.update(stack0=m["stack"].copy(), tray0=tray.pos.copy())
    s0 = _S["stack0"]
    why = ("stack moved" if np.linalg.norm(m["stack"][:2] - s0[:2]) > STACK_MOVE else
           "stack turned" if abs(m["turn"]) > STACK_TURN else
           "stack lifted" if m["stack"][2] - s0[2] > STACK_LIFT else
           "cassette moved" if np.linalg.norm(tray.pos[:2] - _S["tray0"][:2]) > TRAY_MOVE else "")
    if why and not _S["failed"]:
        _S.update(failed=True, why=why)
    _S["last"] = {k: round(1000 * v, 1) for k, v in m["gaps"].items()} | {
        "stack_mm": np.round(1000 * (m["stack"] - s0), 1).tolist(), "failed": _S["why"]}
    if _S["failed"] or _S["touching"]:
        return 0.0
    total = 0.0
    for g in GUIDES:
        gap = m["gaps"][g.name]
        if _S["squeezed"][g.name]:
            continue
        total += 0.5 if gap <= SNUG else 0.25 if gap <= NEAR else 0.0
    return total


# -- scripted demo -----------------------------------------------------------------------------------------------------
# The demo reads nothing at run time: the layout is fixed. Right arm: side guide; left arm: rear guide. Each arm goes
# down around its tab with the fingers open, squeezes it, slides the guide 3 mm past where it meets the stack (the
# stack is against the walls, so the guide stops and the pads slip a little), lets go and leaves.
# DEMO_VARIANT: full (default), nothing, side_only, push (closed fingers push each tab without squeezing it),
# hold (as full but never lets go), shove_stack (pushes the stack itself to the right, toward the side guide).
OVERSHOOT = {"side_guide": 0.003, "rear_guide": 0.008}   # the left arm loses ~2 mm pushing forward (build run 1)
CLEAR = {"right": (0.12, -0.42, TOP + 0.20), "left": (0.12, 0.42, TOP + 0.20)}


def _world(local):
    return (TRAY_XY[0] + local[0], TRAY_XY[1] + local[1], TOP + local[2])


def _guide_plan(arm: str, g: Guide, mode: str) -> list[Waypoint]:
    yaw = 0.0 if g.axis == 1 else math.pi / 2          # 0 closes the fingers along world y
    tab = np.array(_world(g.tab_centre(0.0)))
    z = TOP + TAB_Z1 - 0.003                            # TCP 3 mm below the tab's top: pads on its upper part
    travel = g.stack_edge(np.array([*STACK_LOCAL, 0.0])) - g.face(0.0) + OVERSHOOT[g.name]
    start, end = tab.copy(), tab.copy()
    end[g.axis] += travel
    if mode == "push":
        start[g.axis] -= TAB_HALF_ALONG + 0.015
        steps = [(2.0, (*start[:2], z + 0.06), FINGER_CLOSED, "above, outside the tab"),
                 (1.5, (*start[:2], z), FINGER_CLOSED, "down beside the tab"),
                 (4.0, (*end[:2], z), FINGER_CLOSED, "push without squeezing"),
                 (0.5, (*end[:2], z), FINGER_CLOSED, "hold"),
                 (1.5, (*end[:2], z + 0.08), FINGER_CLOSED, "lift")]
    else:
        steps = [(2.0, (*start[:2], z + 0.06), FINGER_OPEN, "above the tab"),
                 (1.5, (*start[:2], z), FINGER_OPEN, "down around the tab"),
                 (1.0, (*start[:2], z), FINGER_CLOSED, "squeeze the tab"),
                 (0.5, (*start[:2], z), FINGER_CLOSED, "hold"),
                 (4.0, (*end[:2], z), FINGER_CLOSED, "slide the guide to the stack"),
                 (0.5, (*end[:2], z), FINGER_CLOSED, "hold")]
        if mode == "hold":
            return [Waypoint(d, p, f, label, yaw) for d, p, f, label in steps]
        steps += [(1.0, (*end[:2], z), FINGER_OPEN, "let go: the guide locks"),
                  (1.5, (*end[:2], z + 0.08), FINGER_OPEN, "lift")]
    steps.append((1.5, CLEAR[arm], FINGER_OPEN, "move clear"))
    return [Waypoint(d, p, f, label, yaw) for d, p, f, label in steps]


def _shove_stack() -> list[Waypoint]:
    """Right arm, closed fingers on the stack's left part, pushes it 2 cm toward the open side guide."""
    x = TRAY_XY[0] + STACK_LOCAL[0]
    y0 = TRAY_XY[1] + STACK_LOCAL[1] + 0.06
    z = TOP + FLOOR_Z + 0.001 + STACK_SIZE[2] + 0.010
    steps = [(2.0, (x, y0, z + 0.06), FINGER_CLOSED, "above the stack"),
             (1.5, (x, y0, z), FINGER_CLOSED, "down onto the stack's top"),
             (1.5, (x, y0, z - 0.012), FINGER_CLOSED, "press into the top sheet"),
             (3.0, (x, y0 - 0.02, z - 0.012), FINGER_CLOSED, "drag the stack right"),
             (1.5, (x, y0 - 0.02, z + 0.08), FINGER_CLOSED, "lift"),
             (1.5, CLEAR["right"], FINGER_OPEN, "move clear")]
    return [Waypoint(d, p, f, label, 0.0) for d, p, f, label in steps]


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        return {}
    if variant == "shove_stack":
        return {"right": _shove_stack()}
    mode = {"push": "push", "hold": "hold"}.get(variant, "squeeze")
    plan = {"right": _guide_plan("right", SIDE, mode)}
    if variant != "side_only":
        plan["left"] = _guide_plan("left", REAR, mode)
    return plan

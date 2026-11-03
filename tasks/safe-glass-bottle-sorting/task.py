"""Sort four fragile glass bottles onto the shelf matching their color.

The left red shelf and right blue shelf each start with one red and one blue
bottle. A gray staging pad between them is available for swaps. Preserve every
bottle, including the two correctly placed neighbors. Finish with all bottles
upright, still, wholly on the correct shelf, and both grippers withdrawn.
Use run.py: it supplies bottle geometry, contact sensing and episode safety.
"""

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "end"
RADIUS = 0.018
HEIGHT = 0.10
MASS = 0.08
MAX_TILT_DEG = 20.0
MAX_FORCE_N = 40.0
MAX_SPEED = 0.01
MAX_SPIN = 0.10
EDGE_MARGIN = 0.003
RED = (0.80, 0.08, 0.07)
BLUE = (0.06, 0.28, 0.85)

# Shallow open shelves, not enclosed bins: all bottle sides remain accessible.
SHELVES = {
    "red": Box(
        "red_shelf", pos=(0.225, 0.13, TOP + 0.006), size=(0.27, 0.16, 0.012), color=RED
    ),
    "blue": Box(
        "blue_shelf",
        pos=(0.225, -0.13, TOP + 0.012),
        size=(0.27, 0.16, 0.024),
        color=BLUE,
    ),
}
STAGE = Box(
    "staging_pad",
    pos=(0.205, 0.0, TOP + 0.006),
    size=(0.22, 0.065, 0.012),
    color=(0.52, 0.54, 0.58),
)


def surface(shelf):
    return shelf.pos[2] + shelf.size[2] / 2


def bottle(name, x, y, shelf, color):
    # run.py replaces these validated Cylinder specs with compound bottle shapes.
    return Cylinder(
        name,
        pos=(x, y, surface(shelf) + HEIGHT / 2),
        radius=RADIUS,
        height=HEIGHT,
        mass=MASS,
        color=color,
    )


BOTTLES = [
    bottle("blue_swap", 0.16, 0.13, SHELVES["red"], BLUE),
    bottle("red_neighbor", 0.29, 0.13, SHELVES["red"], RED),
    bottle("red_swap", 0.16, -0.13, SHELVES["blue"], RED),
    bottle("blue_neighbor", 0.29, -0.13, SHELVES["blue"], BLUE),
]
OBJECTS = [*SHELVES.values(), STAGE, *BOTTLES]
NAMES = tuple(b.name for b in BOTTLES)
GOALS = {name: name.split("_")[0] for name in NAMES}


def tilt_deg(state):
    q = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (q[0] ** 2 + q[1] ** 2), -1, 1))))


def supported_on(state, shelf):
    low = np.asarray(shelf.pos) - np.asarray(shelf.size) / 2
    high = np.asarray(shelf.pos) + np.asarray(shelf.size) / 2
    return bool(
        np.all(state.low[:2] >= low[:2] + EDGE_MARGIN)
        and np.all(state.high[:2] <= high[:2] - EDGE_MARGIN)
        and abs(state.low[2] - high[2]) <= 0.004
    )


def sorted_bottle(name, state):
    return bool(
        supported_on(state, SHELVES[GOALS[name]])
        and tilt_deg(state) <= 5.0
        and state.resting(MAX_SPEED, MAX_SPIN)
    )


class SafetyMonitor:
    """Episode-local, irreversible safety latch; updated at 240 Hz by run.py.

    Force is the magnitude of the PhysX net normal contact force, not a glass
    material model. The 40 N threshold defines virtual damage for this task.
    """

    def __init__(self):
        self.reasons = set()
        self.peak_force_n = {name: 0.0 for name in NAMES}
        self.max_tilt_deg = {name: 0.0 for name in NAMES}
        self.released = False
        self.armed = False

    def update(self, objects, forces=None):
        if not self.armed:
            return
        for name in NAMES:
            state = objects[name]
            tilt = tilt_deg(state)
            force = float((forces or {}).get(name, 0.0))
            self.max_tilt_deg[name] = max(self.max_tilt_deg[name], tilt)
            self.peak_force_n[name] = max(self.peak_force_n[name], force)
            if not np.all(
                np.isfinite(
                    np.r_[
                        state.pos,
                        state.quat_xyzw,
                        state.lin_vel,
                        state.ang_vel,
                        state.low,
                        state.high,
                    ]
                )
            ) or not np.isfinite(force):
                self.reasons.add(f"{name}:invalid_state")
            if force > MAX_FORCE_N:
                self.reasons.add(f"{name}:virtual_damage")
            if tilt > MAX_TILT_DEG:
                self.reasons.add(f"{name}:tipped")
            if state.low[2] < surface(STAGE) - 0.006:
                self.reasons.add(f"{name}:fell_below_shelves")
            if not (
                0.07 <= state.pos[0] <= 0.38
                and -0.23 <= state.pos[1] <= 0.23
                and state.high[2] <= TOP + 0.32
            ):
                self.reasons.add(f"{name}:outside_workspace")

    @property
    def safe(self):
        return self.armed and not self.reasons


# The stock loader deliberately cannot award credit without instrumented run.py.
MONITOR = SafetyMonitor()


def success(objects):
    return bool(
        MONITOR.safe
        and MONITOR.released
        and all(sorted_bottle(n, objects[n]) for n in NAMES)
    )


def score(objects):
    if not MONITOR.safe or not MONITOR.released:
        return 0.0
    return max(0, sum(sorted_bottle(n, objects[n]) for n in NAMES) - 2) / 2


def distance(objects):
    return float(
        sum(
            np.linalg.norm(objects[n].pos[:2] - np.asarray(SHELVES[GOALS[n]].pos[:2]))
            for n in NAMES
        )
        / len(NAMES)
    )


def demo():
    """Four sequential moves through two staging positions; no arm crosses over."""
    blue, _, red, _ = BOTTLES
    stage_blue = (0.27, 0.0, surface(STAGE) + HEIGHT / 2)
    stage_red = (0.14, 0.0, surface(STAGE) + HEIGHT / 2)
    red_goal = (0.16, 0.13, surface(SHELVES["red"]) + HEIGHT / 2)
    blue_goal = (0.16, -0.13, surface(SHELVES["blue"]) + HEIGHT / 2)
    travel = TOP + 0.18
    parks = {"left": (0.11, 0.26, TOP + 0.16), "right": (0.11, -0.26, TOP + 0.16)}
    plans = {"left": [], "right": []}
    for side, pick, place in [
        ("left", blue.pos, stage_blue),
        ("right", red.pos, stage_red),
        ("left", stage_red, red_goal),
        ("right", stage_blue, blue_goal),
    ]:
        move = pick_place_waypoints(
            pick, (place[0], place[1], place[2] + 0.0015), travel_z=travel, speed=0.75
        )
        move.append(Waypoint(1.5, parks[side], FINGER_OPEN, "withdraw"))
        other = "right" if side == "left" else "left"
        plans[side].extend(move)
        plans[other].append(
            Waypoint(
                sum(w.duration for w in move), parks[other], FINGER_OPEN, "wait clear"
            )
        )
    for side, plan in plans.items():
        plan.append(Waypoint(3, parks[side], FINGER_OPEN, "verify stability"))
    return plans

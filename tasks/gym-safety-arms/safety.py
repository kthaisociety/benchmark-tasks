"""Gym safety arms: the notch detent, the trial record and scoring, and the demo plans, in pure Python (no simulator).

A scaled power rack stands on the station table with a lifter lying on its bench, head toward the robot, under a bar
resting in the J-hooks. Each upright carries a red safety arm that slides up and down and clicks into one of eight
notches. For a bench press the arms belong just below the point where the bar touches the lifter's chest: high
enough to catch a failed rep before it pins the lifter, low enough not to stop a full rep. Both sides at the same
notch. The robot starts with the left arm at the top notch (too high) and the right arm at the bottom (too low).

Metres and seconds. Heights are above the table top, in the rack's frame (x forward, y left, z up).
task.py wraps this for yam_arena; test_task.py checks it offline with a one-dimensional model of each arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# -- geometry -------------------------------------------------------------------------------------------------

RACK_X = 0.32           # world x of the bar, the uprights and the lifter's chest
BAR_W = 0.022           # bar diameter (an octagon across the flats)
BAR_Z = 0.200           # bar centre, resting in the J-hooks
BAR_HALF_LEN = 0.200
BENCH_TOP = 0.040
CHEST_TOP = 0.080       # top of the lifter's chest, under the bar
CONTACT = CHEST_TOP     # where the bar's underside is when it touches the chest
POST_Y = 0.165          # uprights, 3 cm square, inner face at |y| 0.150
POST_H = 0.260
PLATE_Y = (0.181, 0.193)  # two plates each side, outside the uprights; outer face at |y| 0.199
PLATE_W, PLATE_T = 0.090, 0.012

ARM_Y = 0.135           # the safety arms run along x just inside the uprights, under the bar
ARM_W, ARM_T = 0.020, 0.015
ARM_X0, ARM_X1 = 0.170, 0.400    # world x span of each arm: reaching well toward the robot ...
HANDLE_X = 0.1825       # ... so the handle on its near end is 13 cm from the bar: the gripper's housing clears the
                        # bar, the plates and the uprights (with the handle 7 cm away it ran into them)
HANDLE = (0.025, 0.020, 0.030)   # handle size (x, y, z); the fingers close across it, along y

NOTCHES = tuple(round(0.030 + 0.015 * k, 3) for k in range(8))   # arm top heights: 3.0 ... 13.5 cm
TOP_NOTCH, LOW_NOTCH = NOTCHES[-1], NOTCHES[0]
TRAVEL = TOP_NOTCH - LOW_NOTCH   # slider travel; q = 0 at the top notch, -TRAVEL at the bottom

# the safe window for an arm's top: within 1 cm below where the bar touches the chest
WINDOW = (CONTACT - 0.010, CONTACT)
CORRECT = tuple(n for n in NOTCHES if WINDOW[0] <= n < WINDOW[1])   # the one notch inside it: 7.5 cm

START = {"left": TOP_NOTCH, "right": LOW_NOTCH}

# -- the notch detent -----------------------------------------------------------------------------------------

DT = 0.05               # control step (20 Hz)
RATE, DAMPING, ARM_MASS = 150.0, 3.0, 0.05     # the slider drive
SAG = ARM_MASS * 9.81 / RATE                   # added to every target to hold the arm against gravity
UNLOCK = 0.006          # a locked arm pulled this far from its notch (0.9 N at 150 N/m) is released to move ...
STILL_V = 0.0005        # ... and locks again into its nearest notch once it has moved less than this per step ...
STILL_STEPS = 10        # ... for this many steps in a row (0.5 s): a pop pin, worked by a deliberate pull
START_STEPS = 10        # the first 0.5 s, while each arm settles at its start notch, the pins stay locked


def nearest_notch(h: float) -> float:
    return min(NOTCHES, key=lambda n: abs(n - h))


@dataclass
class Detent:
    """One safety arm's pop pin. Locked: the drive holds the arm at its notch, stiffly enough that a brush or a light
    drag (under 0.9 N) moves it less than 6 mm and it springs back. Pulled further, it unlocks and follows wherever it
    is moved (the drive target is its own position), until it has been still for 0.5 s; then it locks into the nearest
    notch."""
    notch: float = TOP_NOTCH
    free: bool = False
    still: int = 0
    last: float = TOP_NOTCH

    def reset(self, notch: float):
        self.notch, self.free, self.still, self.last = notch, False, 0, notch

    def target(self, h: float) -> float:
        if not self.free:
            if abs(h - self.notch) > UNLOCK:
                self.free, self.still = True, 0
        else:
            self.still = self.still + 1 if abs(h - self.last) < STILL_V else 0
            if self.still >= STILL_STEPS:
                self.free, self.notch = False, nearest_notch(h)
        self.last = h
        return h if self.free else self.notch


def h_to_q(h: float) -> float:
    return h - TOP_NOTCH


def q_to_h(q: float) -> float:
    return q + TOP_NOTCH


# -- the trial record and the score ---------------------------------------------------------------------------

def classify(h: float) -> str:
    """'low' (the bar would land on the chest first: it can pin the lifter), 'ok' (within 1 cm below the chest
    contact), or 'high' (it stops the bar before it reaches the chest)."""
    if h < WINDOW[0]:
        return "low"
    if h < WINDOW[1]:
        return "ok"
    return "high"


def score_heights(left: float, right: float, rack_moved: bool = False) -> float:
    """0 if the rack moved or either arm is low (unsafe). Otherwise 1.0 both in the window; 0.5 one in it and the other
    high; 0.25 both high (safe, but no full rep possible)."""
    if rack_moved:
        return 0.0
    kinds = (classify(left), classify(right))
    if "low" in kinds:
        return 0.0
    return {2: 1.0, 1: 0.5, 0: 0.25}[kinds.count("ok")]


@dataclass
class Trial:
    rack_moved: bool = False
    heights: dict = field(default_factory=lambda: dict(START))
    pins: dict = field(default_factory=lambda: {side: Detent() for side in START})
    steps: int = 0

    def reset(self):
        self.rack_moved = False
        self.steps = 0
        self.heights = dict(START)
        for side, n in START.items():
            self.pins[side].reset(n)

    def observe(self, left: float, right: float, rack_moved: bool = False):
        """One control step: the arms' measured heights in; returns their drive targets (heights)."""
        self.rack_moved |= rack_moved
        self.heights = {"left": left, "right": right}
        self.steps += 1
        if self.steps <= START_STEPS:      # the arms are still settling at their start notches: stay locked
            return START["left"], START["right"]
        return self.pins["left"].target(left), self.pins["right"].target(right)

    def score(self) -> float:
        return score_heights(self.heights["left"], self.heights["right"], self.rack_moved)


# -- the demo: each arm grips its safety arm's handle and moves it straight up or down -------------------------

TIP = 0.008             # the waypoint TCP sits this far above the finger-pad tips
PAD_LEN = 0.030         # the pads reach this far up from the tips; the handle's top must stay under the palm
GRIP_UP = 0.007         # pad tips this far above the handle's bottom (the arm's top)
YAW = 0.0               # fingers close along y, across the handle
OPEN = -0.025           # 50 mm opening: 15 mm clear of the 20 mm handle each side
SQUEEZE = 0.008
GRIP = -(HANDLE[1] - SQUEEZE) / 2
FINGER_OPEN = -0.047    # full opening (task.py passes yam_arena's own constant)
CLOSE_T = 1.5
CLEAR = 0.050           # pad tips this far above the handle's top when approaching or leaving
PARK = {"left": (0.20, 0.36, 0.25), "right": (0.20, -0.36, 0.25)}


def tcp_for(arm_top: float) -> float:
    """TCP height that grips the handle on an arm whose top is at arm_top."""
    return arm_top + GRIP_UP + TIP


def side_plan(side: str, to: float | None, top: float = 0.0, release: bool = True, delay: float = 0.0):
    """Waypoints (duration, (x, y, z), finger, label, yaw) for one arm: over the handle, down, grip, move the safety
    arm to `to`, let go, lift clear and park. `to=None` parks without touching it."""
    y = ARM_Y if side == "left" else -ARM_Y
    px, py, pz = PARK[side]
    park = (1.5, (px, py, top + pz), FINGER_OPEN, "park clear of the camera", YAW)
    if to is None:
        return [(2.0, (px, py, top + pz), FINGER_OPEN, "stay parked", YAW), (30.0, park[1], FINGER_OPEN, "wait", YAW)]
    h0 = START[side]
    above = top + h0 + HANDLE[2] + CLEAR + TIP
    plan = []
    if delay:
        plan.append((delay, (px, py, top + pz), FINGER_OPEN, "wait", YAW))
    plan += [(2.0, (HANDLE_X, y, above), OPEN, "over the handle", YAW),
             (1.2, (HANDLE_X, y, top + tcp_for(h0)), OPEN, "down around the handle", YAW),
             (0.3, (HANDLE_X, y, top + tcp_for(h0)), OPEN, "settle", YAW),
             (CLOSE_T, (HANDLE_X, y, top + tcp_for(h0)), GRIP, "grip the handle", YAW),
             (max(1.0, abs(to - h0) / 0.05), (HANDLE_X, y, top + tcp_for(to)), GRIP, "move the safety arm", YAW),
             (0.6, (HANDLE_X, y, top + tcp_for(to)), GRIP, "hold at the notch", YAW)]
    if release:
        plan += [(CLOSE_T, (HANDLE_X, y, top + tcp_for(to)), OPEN, "let go", YAW),
                 (1.0, (HANDLE_X, y, top + to + HANDLE[2] + CLEAR + TIP), OPEN, "lift clear", YAW), park]
    else:
        plan.append((30.0, (HANDLE_X, y, top + tcp_for(to)), GRIP, "keep hold", YAW))
    return plan


OK = CORRECT[0]
VARIANTS = {   # (left target, right target)
    "correct": (OK, OK),
    "nothing": (None, None),
    "right_only": (None, OK),                 # left left at the top: one side right, the other blocks the rep
    "both_top": (None, TOP_NOTCH),            # everything to the top: safe, but no full rep
    "both_low": (LOW_NOTCH, None),            # everything to the bottom: the bar would pin the lifter
    "one_low": (OK - 0.015, OK - 0.015),      # one notch too low on both sides
    "one_high": (OK + 0.015, OK + 0.015),     # one notch too high on both sides
    "uneven": (OK, OK + 0.015),               # one side right, the other one notch high
    "held_between": (OK + 0.005, OK + 0.005), # let go 5 mm above the notch, off any notch: each locks into 7.5 cm
}
EXPECTED = {"correct": 1.0, "nothing": 0.0, "right_only": 0.5, "both_top": 0.25, "both_low": 0.0, "one_low": 0.0,
            "one_high": 0.25, "uneven": 0.5, "held_between": 1.0}


def demo_plan(variant: str = "correct", top: float = 0.0):
    left, right = VARIANTS[variant]
    return {"left": side_plan("left", left, top), "right": side_plan("right", right, top, delay=0.5)}


# -- a one-dimensional model of each safety arm and its gripper, for offline checks ---------------------------

LAG = 0.05              # the drive's time constant, s


def at(plan, t):
    """Linear interpolation of a plan at time t: (pos, finger, label)."""
    t0, prev_p, prev_f = 0.0, None, FINGER_OPEN
    for dur, pos, f, label, _ in plan:
        if t <= t0 + dur:
            s = (t - t0) / dur if dur > 0 else 1.0
            p = pos if prev_p is None else tuple(a + s * (b - a) for a, b in zip(prev_p, pos))
            return p, prev_f + s * (f - prev_f), label
        t0 += dur
        prev_p, prev_f = pos, f
    return prev_p, prev_f, "end"


def plan_duration(plan) -> float:
    return sum(d for d, *_ in plan)


def simulate(plans, seconds: float = 30.0, top: float = 0.0, lag: float = LAG):
    """Run both arms against the plans. A safety arm is held once the fingers are closed past halfway between the
    opening and the grip with the pad tips on its handle; then it follows the TCP. Otherwise its drive pulls it toward
    the detent target. Returns the trial and a log of (t, left, right, held_left, held_right)."""
    trial = Trial()
    trial.reset()
    h = dict(START)
    held = {"left": False, "right": False}
    closed = (OPEN + GRIP) / 2
    log = []
    for k in range(int(round(seconds / DT))):
        t = k * DT
        for side in ("left", "right"):
            plan = plans.get(side) or []
            pos, f, _ = at(plan, t) if plan else (None, FINGER_OPEN, "idle")
            y = ARM_Y if side == "left" else -ARM_Y
            on = (pos is not None and abs(pos[0] - HANDLE_X) < 0.012 and abs(pos[1] - y) < 0.012)
            tips = None if pos is None else pos[2] - top - TIP
            if on and not held[side] and f >= closed and abs(tips - (h[side] + GRIP_UP)) < 0.010:
                held[side] = True
            elif held[side] and (f < closed or not on):
                held[side] = False
            if held[side]:
                h[side] = min(TOP_NOTCH, max(LOW_NOTCH, tips - GRIP_UP))
        tl, tr = trial.observe(h["left"], h["right"])
        for side, tgt in (("left", tl), ("right", tr)):
            if not held[side]:
                h[side] += (tgt - h[side]) * min(1.0, DT / lag)
        log.append((t, h["left"], h["right"], held["left"], held["right"]))
    return trial, log

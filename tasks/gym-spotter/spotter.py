"""Gym spotter: the scripted lifter, the scoring and the demo plans, in pure Python (no simulator needed).

A scaled-down smith machine stands on the station table. A lifter lies on the bench with their head toward the robot,
and the bar rides a vertical guide over their chest. The lifter presses a set: three easy reps, a slower fourth, a
grinding fifth, and on the sixth the bar stalls a few centimetres off the chest, shakes, and sinks back down. The
robot is the spotter. It should leave the bar alone while the lifter is moving it, take hold when the sixth rep stalls,
and help it up to lockout, where the lifter racks it.

Everything here is in metres and seconds. Bar heights are measured from the bottom stop (bar resting on the chest).
task.py wraps this for yam_arena; test_task.py checks it offline with a one-dimensional model of the bar and gripper.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# -- geometry (part frame: origin on the table top under the bar's centre line, x forward, y left, z up) --------

BAR_X = 0.32            # world x of the bar's axis
TRAVEL = 0.100          # bar travel from the chest to lockout
BAR_W = 0.022           # bar: an octagon 22 mm across the flats, along y
BAR_HALF_LEN = 0.195
BENCH_TOP = 0.040
CHEST_TOP = 0.080       # top of the lifter's chest
BAR_LOW = CHEST_TOP + BAR_W / 2 + 0.000   # bar centre at the bottom stop: resting on the chest
BAR_HIGH = BAR_LOW + TRAVEL               # bar centre at lockout (the slider's rest stop)
PLATE_W, PLATE_T = 0.090, 0.012           # octagonal plates, two each side
PLATE_Y = (0.140, 0.153)
HAND_Y = 0.115          # the lifter's hands on the bar, either side
POST_Y = 0.185          # the machine's uprights (3 cm square): nothing past |y| 0.20, where the arms park
POST_H = 0.260

# -- the lifter's set -------------------------------------------------------------------------------------------

DT = 0.05               # control step (20 Hz)
START_HOLD = 1.0        # bar held at lockout after the unrack
DOWN = 1.0              # every descent
BOTTOM = 0.15           # touch-and-go at the chest
EASY_UP = 0.9           # reps 1-3
REP4_UP = 1.3
REP5_UP = 3.0           # rep 5 grinds through a sticking point but finishes
TOP_PAUSE = 0.6         # between easy reps
BREATH = 1.5            # at lockout after reps 4 and 5
FAIL_RISE = 1.6         # rep 6: up to STALL_H ...
STALL_H = 0.035
FAIL_GRIND = 2.5        # ... shaking there ...
SHAKE_AMP, SHAKE_HZ = 0.002, 3.0
SINK_V = STALL_H / 2.0  # ... then sinking back to the chest in 2 s

FOLLOW = 0.005          # a bar lifted this far above where the lifter has it counts as lifted by the spotter
CARRY = 0.001           # then the lifter's target stays this far under it: 0.5 N on the spotter (the arm pushes about 2 N)
                        # (a 5 mm follow, 2.5 N, lost the bar in the simulator at 8 cm)
LOCK_TOL = 0.010        # within this of lockout counts as locked out
DEV_TOL = 0.020         # bar this far off the lifter's own path ...
DEV_STEPS = 5           # ... for this many steps in a row (0.25 s) is interference
HOLDBACK = 0.008        # in reps 5 and 6 the lifter can't get further than this ahead of a bar that is held back
DROP_H = 0.008          # bar this close to the chest after the stall means it came down on the lifter
HELD_H = 0.020          # bar at least this far off the chest at the end
SETTLE_STEPS = 10       # deviations are not counted in the first 0.5 s


def smooth(s: float) -> float:
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


def grind_profile(s: float) -> float:
    """Rep 5's ascent: quick to 40%, a crawl through the sticking point to 55%, then up to lockout."""
    s = min(1.0, max(0.0, s))
    if s < 0.2:
        return 0.40 * smooth(s / 0.2)
    if s < 0.8:
        return 0.40 + 0.15 * (s - 0.2) / 0.6
    return 0.55 + 0.45 * smooth((s - 0.8) / 0.2)


def invert(profile, h: float) -> float:
    """The phase fraction at which a monotone profile first reaches h (0..1)."""
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if profile(mid) < h:
            lo = mid
        else:
            hi = mid
    return hi


@dataclass(frozen=True)
class Phase:
    kind: str          # hold | down | up | grind | fail
    dur: float
    rep: int = 0
    up_dur: float = 0.0


def schedule() -> list[Phase]:
    """The set, phase by phase. The fail phase never ends by itself."""
    p = [Phase("hold", START_HOLD)]
    ups = [EASY_UP, EASY_UP, EASY_UP, REP4_UP, REP5_UP]
    for rep, up in enumerate(ups, start=1):
        p += [Phase("down", DOWN, rep), Phase("bottom", BOTTOM, rep)]
        p.append(Phase("grind" if rep == 5 else "up", up, rep))
        p.append(Phase("hold", BREATH if rep >= 4 else TOP_PAUSE, rep))
    p += [Phase("down", DOWN, 6), Phase("bottom", BOTTOM, 6), Phase("fail", math.inf, 6)]
    return p


def phase_starts() -> dict[str, float]:
    """Nominal start times (unassisted) of a few landmarks, for the demo plans and the README."""
    t, out = 0.0, {}
    for ph in schedule():
        key = f"{ph.kind}{ph.rep}"
        out.setdefault(key, t)
        t += ph.dur if math.isfinite(ph.dur) else 0.0
    out["stall"] = out["fail6"] + FAIL_RISE
    out["sink"] = out["stall"] + FAIL_GRIND
    out["pinned"] = out["sink"] + STALL_H / SINK_V
    return out


class Lifter:
    """The scripted lifter. ``step(bar_h)`` takes the bar's measured height (0 on the chest, TRAVEL at lockout) and
    returns where the lifter wants the bar this step (the joint target before sag compensation)."""

    def __init__(self):
        self.phases = schedule()
        self.reset()

    def reset(self):
        self.i, self.tau, self.L = 0, 0.0, TRAVEL
        self.fail_t = 0.0
        self.racked = False
        self.lifted_by_spotter = False
        self.carried = False       # the spotter has lifted the bar in this rep: follow it closely

    @property
    def phase(self) -> Phase:
        return self.phases[self.i]

    def _next(self):
        self.i += 1
        self.tau = 0.0
        self.carried = False

    def own_path(self) -> bool:
        """True while the lifter controls the bar alone and any deviation is interference."""
        return self.racked or self.phase.kind in ("hold", "down", "bottom", "up")

    def step(self, bar_h: float) -> float:
        ph = self.phase
        if self.racked:
            self.L = TRAVEL
            return self.L
        if ph.kind == "hold":
            self.L = TRAVEL
        elif ph.kind == "down":
            self.L = TRAVEL * (1 - smooth(self.tau / ph.dur))
        elif ph.kind == "bottom":
            self.L = 0.0
        elif ph.kind == "up":
            self.L = TRAVEL * smooth(self.tau / ph.dur)
        elif ph.kind == "grind":
            # the lifter's own progress, held back by a bar that is held back, carried along by a bar that is lifted
            own = TRAVEL * grind_profile(self.tau / ph.dur)
            if bar_h - FOLLOW > own or (self.carried and bar_h - CARRY > own):
                self.carried = True
                self.tau = invert(grind_profile, (bar_h - CARRY) / TRAVEL) * ph.dur
                own = bar_h - CARRY
            elif own > bar_h + HOLDBACK:
                self.tau = max(0.0, self.tau - DT)     # can't push past a held bar: no progress this step
                own = TRAVEL * grind_profile(self.tau / ph.dur)
            self.L = min(TRAVEL, own)
            if self.L >= TRAVEL - LOCK_TOL:
                self.L = TRAVEL
                self._next()
                return self.L
        elif ph.kind == "fail":
            t = self.fail_t
            if t < FAIL_RISE:
                v = STALL_H / FAIL_RISE
            elif t < FAIL_RISE + FAIL_GRIND:
                v = 0.0
            else:
                v = -SINK_V
            self.L = max(0.0, min(self.L + v * DT, bar_h + HOLDBACK)) if t > 0 else 0.0
            if bar_h - FOLLOW > self.L or (self.carried and bar_h - CARRY > self.L):
                self.carried = True
                self.L = bar_h - CARRY
                if bar_h > STALL_H + SHAKE_AMP + 0.005 or t >= FAIL_RISE:
                    self.lifted_by_spotter = True
            self.fail_t += DT
            if bar_h >= TRAVEL - LOCK_TOL:
                self.racked = True
                self.L = TRAVEL
                return self.L
            shake = SHAKE_AMP * math.sin(2 * math.pi * SHAKE_HZ * t) if FAIL_RISE <= t < FAIL_RISE + FAIL_GRIND else 0.0
            return max(0.0, min(TRAVEL, self.L + shake))
        self.tau += DT
        if self.tau >= ph.dur - 1e-9:
            self._next()
        return self.L

    def stage(self) -> str:
        """rep1..rep6, racked; the fail rep reports rise / stall / sink / pinned."""
        if self.racked:
            return "racked"
        ph = self.phase
        if ph.kind != "fail":
            return f"rep{max(1, ph.rep)}"
        t = self.fail_t
        if t < FAIL_RISE:
            return "rise"
        if t < FAIL_RISE + FAIL_GRIND:
            return "stall"
        return "pinned" if self.L <= 0.0 else "sink"


# -- the trial record and the score ----------------------------------------------------------------------------

@dataclass
class Trial:
    lifter: Lifter = field(default_factory=Lifter)
    steps: int = 0
    dev_run: int = 0
    interfered: bool = False
    rack_moved: bool = False
    dropped: bool = False
    racked_in_fail: bool = False
    bar_h: float = TRAVEL
    last_target: float = TRAVEL

    def reset(self):
        self.lifter.reset()
        self.steps = self.dev_run = 0
        self.interfered = self.rack_moved = self.dropped = self.racked_in_fail = False
        self.bar_h = self.last_target = TRAVEL

    def observe(self, bar_h: float, rack_moved: bool = False) -> float:
        """One control step: the measured bar height in, the lifter's target out. Latches every violation."""
        self.steps += 1
        self.bar_h = bar_h
        self.rack_moved |= rack_moved
        lf = self.lifter
        before_racked = lf.racked
        expected = self.last_target
        own = lf.own_path() or before_racked
        # interference: the bar held off the lifter's path, either way, while the lifter moves it alone (the warm-up
        # hold, every descent, reps 1-4, and after the rack)
        off = (bar_h - expected) if own else 0.0    # in reps 5 and 6 the lifter goes with the bar either way
        if self.steps > SETTLE_STEPS and abs(off) > DEV_TOL:
            self.dev_run += 1
        else:
            self.dev_run = 0
        if self.dev_run >= DEV_STEPS:
            self.interfered = True
        if lf.phase.kind == "fail" and not lf.racked and lf.fail_t >= FAIL_RISE and bar_h < DROP_H:
            self.dropped = True
        target = lf.step(bar_h)
        if lf.racked and not before_racked:
            self.racked_in_fail = True
        self.last_target = target
        return target

    def score(self) -> float:
        """0 if the machine was moved or the bar was pushed or held off the lifter's path while they moved it.
        1.0 the failed sixth rep was caught before the bar came back down on the chest and brought to lockout
        (racked); 0.5 brought to lockout only after it came down on the chest; 0.25 the bar caught and held at least 2 cm off the chest at the end, never brought to
        lockout; otherwise 0."""
        if self.rack_moved or self.interfered:
            return 0.0
        lf = self.lifter
        if self.racked_in_fail:
            return 0.5 if self.dropped else 1.0
        if lf.phase.kind == "fail" and lf.lifted_by_spotter and not self.dropped and self.bar_h >= HELD_H:
            return 0.25
        return 0.0

    def success(self) -> bool:
        return self.score() >= 1.0


# -- the demo (right arm, open loop on the lifter's fixed timing) ------------------------------------------------

YAW = math.pi / 2              # right arm: fingers close along x, across the bar
GRIP_Y = -0.050                # between the lifter's right hand (y -0.115) and the centre
TIP = 0.008                    # the waypoint TCP sits this far above the finger-pad tips
SQUEEZE = 0.008
GRIP = -(BAR_W - SQUEEZE) / 2  # finger target when gripping the bar, m
OPEN = -0.027                  # 54 mm opening: 16 mm clear of the bar each side
FINGER_OPEN = -0.047           # full opening (yam_arena's FINGER_OPEN; task.py passes the real constant)
CLOSE_T = 1.5                  # the gripper opens and closes over 1.5 s
HOVER = BAR_HIGH + BAR_W / 2 + 0.040 + TIP   # pad tips 4 cm above the bar's top at lockout
PARK = (0.20, -0.36, BAR_HIGH + 0.10)
LIFT_TO = TRAVEL + 0.006       # aim past lockout: under load the arm trails its target by about a centimetre


def tcp_for(bar_h: float) -> float:
    """TCP height that puts the pad tips at the bar's centre line when the bar is at bar_h."""
    return BAR_LOW + bar_h + TIP


def _rep5_catch(T):
    """Where and when to take hold in rep 5's sticking point: the fingers arrive as the bar passes 40% of its travel
    and are set where the bar will be when they bite."""
    s_arrive = invert(grind_profile, 0.40) + 0.02
    s_hold = s_arrive + (0.3 + CLOSE_T / 2) / REP5_UP     # the fingers bite halfway through the close
    return TRAVEL * grind_profile(s_hold), T["grind5"] + s_arrive * REP5_UP


def demo_plan(variant: str = "correct", top: float = 0.0):
    """Waypoints (duration, (x, y, z), finger, label, yaw) for the right arm, z relative to ``top`` (the table top).
    Variants: correct | nothing | late | rep5 | takeover | halfway | block."""
    T = phase_starts()
    x, y = BAR_X, GRIP_Y
    plan: list = []
    clock = [0.0]

    def go(dur, z, finger, label):
        plan.append((dur, (x, y, top + z), finger, label, YAW))
        clock[0] += dur

    def wait_until(t, z, finger, label):
        if t - clock[0] < 0.05:
            raise ValueError(f"{variant}: '{label}' is {clock[0] - t:.2f} s late")
        go(t - clock[0], z, finger, label)

    def park():
        go(1.0, HOVER + 0.03, OPEN, "lift clear")
        plan.append((1.5, (PARK[0], PARK[1], top + PARK[2]), FINGER_OPEN, "park clear of the camera", YAW))

    def catch_and_lift(h_catch, t_arrive, lift_to=LIFT_TO, release=True):
        wait_until(t_arrive - 1.0, HOVER, OPEN, "wait over the bar")
        go(1.0, tcp_for(h_catch), OPEN, "lower around the bar")
        go(0.3, tcp_for(h_catch), OPEN, "settle")
        go(CLOSE_T, tcp_for(h_catch), GRIP, "take hold")
        if lift_to is None:
            return
        go(1.5, tcp_for(lift_to), GRIP, "help it up to lockout")
        go(0.5, tcp_for(lift_to), GRIP, "settle at lockout")
        if release:
            go(CLOSE_T, tcp_for(lift_to), OPEN, "let go: the lifter racks it")

    if variant == "nothing":
        return [(2.0, (PARK[0], PARK[1], top + PARK[2]), FINGER_OPEN, "park", YAW),
                (40.0, (PARK[0], PARK[1], top + PARK[2]), FINGER_OPEN, "stay parked", YAW)]
    if variant == "correct":
        # watch from the parking spot, clear of the camera, through reps 1-5; come over the bar as rep 6 goes down
        plan.append((2.0, (PARK[0], PARK[1], top + PARK[2]), OPEN, "watch from the side", YAW))
        clock[0] += 2.0
        plan.append((T["down6"] - clock[0], (PARK[0], PARK[1], top + PARK[2]), OPEN, "watch reps 1-5", YAW))
        clock[0] = T["down6"]
        go(1.0, HOVER + 0.05, OPEN, "over the bar, high over the upright")
        go(0.5, HOVER, OPEN, "down to the hover, hands ready")
        catch_and_lift(STALL_H, T["stall"] + 0.2)
        park()
        return plan
    go(2.0, HOVER, OPEN, "over the bar, hands ready")
    if variant == "halfway":
        catch_and_lift(STALL_H, T["stall"] + 0.2, lift_to=None)
        go(40.0, tcp_for(STALL_H), GRIP, "hold it off the chest")
    elif variant == "late":
        catch_and_lift(0.0, T["pinned"] + 0.5)
        park()
    elif variant == "rep5":
        # take hold in rep 5's sticking point and help it to lockout too, then catch rep 6 as in `correct`
        catch_and_lift(*_rep5_catch(T), lift_to=TRAVEL - 0.004)    # just to the top: the lifter finishes rep 5
        go(1.0, HOVER, OPEN, "back over the bar")
        # the help ended rep 5 early, so rep 6 comes sooner: time it on the lifter model
        catch_and_lift(STALL_H, _stall_after(list(plan), top) + 0.2)
        park()
    elif variant == "takeover":
        # the same rep-5 help, but then keeps hold at lockout, so the lifter can't lower it for rep 6
        catch_and_lift(*_rep5_catch(T), release=False)
        go(30.0, tcp_for(LIFT_TO), GRIP, "keep hold at lockout")
    elif variant == "block":
        # fingers shut, resting in the bar's path halfway down: the next rep runs into them
        go(1.5, HOVER, GRIP, "close the fingers")
        wait_until(T["down2"] + 0.5, HOVER, GRIP, "wait")
        go(0.4, tcp_for(TRAVEL * 0.5) + BAR_W / 2, GRIP, "into the bar's path")
        go(30.0, tcp_for(TRAVEL * 0.5) + BAR_W / 2, GRIP, "stay there")
    else:
        raise ValueError(f"unknown variant {variant!r}")
    return plan


def _stall_after(plan, top: float) -> float:
    """When rep 6 stalls if the arm follows ``plan`` and then waits over the bar."""
    _, log = simulate(plan + [(60.0, (BAR_X, GRIP_Y, top + HOVER), OPEN, "wait", YAW)], seconds=40.0, top=top)
    return next(t for t, _, _, stage, _, _ in log if stage == "stall")


EXPECTED = {"correct": 1.0, "nothing": 0.0, "late": 0.5, "rep5": 1.0, "takeover": 0.0, "halfway": 0.25,
            "block": 0.0}


# -- a one-dimensional model of the bar and the gripper, for offline checks ---------------------------------------

PAD_LEN = 0.030        # finger pads reach this far up from the tips: a bar above that hits the palm
LAG = 0.04             # the drive's time constant (damping / stiffness), s


def finger_at(plan, t):
    """Linear interpolation of the plan at time t: (tcp_z, finger, label). Before the first waypoint the arm is up
    and clear (z None)."""
    t0, prev_z, prev_f = 0.0, None, FINGER_OPEN
    for dur, pos, f, label, _ in plan:
        if t <= t0 + dur:
            s = (t - t0) / dur if dur > 0 else 1.0
            z = pos[2] if prev_z is None else prev_z + s * (pos[2] - prev_z)
            return z, prev_f + s * (f - prev_f), label, pos
        t0 += dur
        prev_z, prev_f = pos[2], f
    if not plan:
        return None, FINGER_OPEN, "idle", None
    return prev_z, prev_f, "end", plan[-1][1]


def simulate(plan, seconds: float = 35.0, top: float = 0.0, lag: float = LAG):
    """Run the lifter against the plan. The gripper holds the bar once its fingers are closed past halfway between
    the opening and the grip with the bar between the pads; a closed gripper in the bar's path blocks it. Returns the
    trial and a log of (t, bar_h, target, stage, held)."""
    trial = Trial()
    trial.reset()
    bar, target, held, log = TRAVEL, TRAVEL, False, []
    closed_at = (OPEN + GRIP) / 2
    n = int(round(seconds / DT))
    for k in range(n):
        t = k * DT
        z, f, label, pos = finger_at(plan, t)
        over = pos is not None and abs(pos[0] - BAR_X) < 0.03 and abs(pos[1] - GRIP_Y) < 0.03
        tips = None if z is None else z - top - TIP
        if over and tips is not None:
            between = abs(tips - (BAR_LOW + bar)) <= 0.012
            if not held and f >= closed_at and between:
                held = True
            elif held and f < closed_at:
                held = False
        else:
            held = False
        if held:
            bar = min(TRAVEL, max(0.0, tips - BAR_LOW))
        else:
            bar += (target - bar) * min(1.0, DT / lag)
            if over and tips is not None:
                straddling = f <= closed_at
                ceiling = tips + PAD_LEN if straddling else tips   # palm, or the shut fingertips
                bar = min(bar, ceiling - BAR_W / 2 - BAR_LOW)
            bar = min(TRAVEL, max(0.0, bar))
        target = trial.observe(bar)
        log.append((t, bar, target, trial.lifter.stage(), held, label))
        if trial.success():
            break
    return trial, log

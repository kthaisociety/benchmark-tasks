"""On the station table, in front of the right arm: a valve with an electric actuator on top of it. The actuator has
a flat black handwheel on top, and a motor housing running off to the left with an orange hand/auto (declutch) paddle
on it. Beside the paddle, a white position indicator strip runs from a green OPEN end to a red CLOSED end; its black
pointer is at the green end. The actuator has lost power.

The handwheel is not coupled in motor mode: it turns freely and the valve does not move. Holding the paddle down while
the handwheel is turned engages the clutch the next time a clutch dog lines up (every 120 degrees of the wheel). The
paddle then latches down, and turning the wheel clockwise (seen from above) closes the valve: one turn from the
engaging point to a closed stop, with an open stop at the engaging point.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "The valve actuator has lost power. Close the valve by hand: hold the orange lever down and turn the handwheel "
    "until the lever stays down by itself, then turn the handwheel clockwise until the indicator's pointer reaches "
    "the red CLOSED end."
)

GREY, BLUE, BLACK = (0.55, 0.57, 0.60), (0.30, 0.40, 0.52), (0.12, 0.12, 0.13)
ORANGE, WHITE, GREEN, RED = (0.93, 0.40, 0.07), (0.94, 0.94, 0.94), (0.15, 0.65, 0.25), (0.85, 0.12, 0.12)

# The part frame: origin on the table on the handwheel's axis, axes as the world's (x away from the robot, y left).
VX, VY = 0.30, -0.13
HUB_Z = 0.135                      # spokes' centre height above the table
R_RIM = 0.075                      # rim centre radius
LEVER_AT = (-0.020, 0.170)         # paddle centre (part x, y)
LEVER_TOP = 0.095                  # paddle top above the table, at rest
LEVER_TRAVEL = 0.012               # 12 mm
LEVER_K = 125.0                    # N/m: 1.5 N at full travel
LEVER_MASS = 0.03
SAG = LEVER_MASS * 9.81 / LEVER_K  # spring rest offset so the paddle sits at its top stop
DOG0, DOG_PITCH = math.radians(50.0), math.radians(120.0)  # nearest dog 50 deg clockwise, then every 120 deg
CLOSE_TURNS = 1.0
CLOSED = CLOSE_TURNS * 2 * math.pi
SHUT_TOL = math.radians(5.0)       # closed: within 5 degrees of the closed stop
STOP_K = 30.0                      # N m/rad: the emulated end stops, only while the wheel is past one
WHEEL_DAMPING = 0.05               # N m s/rad
IND_AT = (0.025, 0.075)            # indicator strip: part x, y of its OPEN (green) end
IND_LEN = 0.060                    # pointer travel, OPEN to CLOSED, along +y
IND_Z = 0.060                      # housing top
SUCCESS_HOLD_S = 3.0

BODY = [
    parts.box((0.07, 0.07, 0.07), at=(0.0, 0.0, 0.035), color=GREY),             # valve body
    parts.cylinder(0.0175, 0.12, at=(0.0, 0.0, 0.030), color=GREY, axis="x"),   # pipe stubs, front and back
    parts.box((0.13, 0.12, 0.045), at=(0.0, 0.0, 0.0925), color=BLUE),           # actuator gearcase
    parts.cylinder(0.010, 0.016, at=(0.0, 0.0, 0.123), color=GREY),              # handwheel shaft
    parts.box((0.10, 0.15, IND_Z), at=(0.0, 0.135, IND_Z / 2), color=BLUE),      # motor housing, to the left
    parts.box((0.012, IND_LEN + 0.012, 0.002), at=(IND_AT[0], IND_AT[1] + IND_LEN / 2, IND_Z + 0.001), color=WHITE),
    parts.box((0.013, 0.010, 0.0022), at=(IND_AT[0], IND_AT[1], IND_Z + 0.0011), color=GREEN),
    parts.box((0.013, 0.010, 0.0022), at=(IND_AT[0], IND_AT[1] + IND_LEN, IND_Z + 0.0011), color=RED),
]
N_RIM = 32
_rim = [
    parts.box((0.012, 0.012, 0.012),
              at=(R_RIM * math.cos(2 * math.pi * k / N_RIM), R_RIM * math.sin(2 * math.pi * k / N_RIM), HUB_Z),
              color=BLACK)
    for k in range(N_RIM)
]
_L = R_RIM - 0.006
_spokes = [parts.box((_L, 0.008, 0.012), at=(sx * _L / 2, 0.0, HUB_Z), color=BLACK) for sx in (1, -1)] + [
    parts.box((0.008, _L, 0.012), at=(0.0, sy * _L / 2, HUB_Z), color=BLACK) for sy in (1, -1)
]
WHEEL = parts.Hinge(
    "wheel", pivot=(0.0, 0.0, HUB_Z), axis=(0.0, 0.0, -1.0),  # + = clockwise from above
    lower=-8 * math.pi, upper=8 * math.pi, mass=0.3, damping=WHEEL_DAMPING,
    shapes=[parts.cylinder(0.016, 0.016, at=(0.0, 0.0, HUB_Z), color=BLACK), *_rim, *_spokes],
)
LEVER = parts.Slider(
    "lever", axis=(0.0, 0.0, -1.0), travel=LEVER_TRAVEL, mass=LEVER_MASS, spring=LEVER_K, damping=2.0,
    shapes=[
        parts.box((0.050, 0.050, 0.008), at=(LEVER_AT[0], LEVER_AT[1], LEVER_TOP - 0.004), color=ORANGE),
        parts.box((0.010, 0.010, 0.012), at=(LEVER_AT[0], LEVER_AT[1], LEVER_TOP - 0.014), color=ORANGE),
    ],
)
POINTER = parts.Slider(
    "pointer", axis=(0.0, 1.0, 0.0), travel=IND_LEN, mass=0.01, spring=40.0, damping=1.0,
    shapes=[parts.box((0.016, 0.004, 0.004), at=(IND_AT[0], IND_AT[1], IND_Z + 0.004), color=BLACK)],
)
ACTUATOR = parts.Part("actuator", pos=(VX, VY, TOP), shapes=BODY, joints=[WHEEL, LEVER, POINTER])
OBJECTS = [ACTUATOR]

# -- clutch, stops and indicator (run after every control step) -------------------------------------------------------
STATE = {"engaged": False, "q0": 0.0, "prev": 0.0, "q": 0.0, "stop": None}
TRACE = os.environ.get("AMO_TRACE")


def _crossed_dog(a: float, b: float) -> bool:
    lo, hi = min(a, b), max(a, b)
    k = math.ceil((lo - DOG0) / DOG_PITCH)
    return DOG0 + k * DOG_PITCH <= hi


def _set_stiffness(sim, joint: str, k: float) -> None:
    import torch

    asset = sim.env.unwrapped.scene["actuator"]
    ids = asset.find_joints([joint], preserve_order=True)[0]
    asset.write_joint_stiffness_to_sim_index(
        stiffness=torch.tensor([[float(k)]], dtype=torch.float32, device=sim.env.unwrapped.device), joint_ids=ids
    )


@parts.on_reset
def _reset(sim):
    STATE.update(engaged=False, q0=0.0, prev=0.0, q=0.0, stop=None)
    _set_stiffness(sim, "wheel", 0.0)
    sim.set_target("actuator", "lever", -SAG)
    sim.set_target("actuator", "pointer", 0.0)


@parts.on_step
def _clutch(sim):
    s = sim.states()["actuator"]
    q, lever = float(s.joints["wheel"]), float(s.joints["lever"])
    if not STATE["engaged"] and lever >= 0.9 * LEVER_TRAVEL and _crossed_dog(STATE["prev"], q):
        STATE.update(engaged=True, q0=q)
        sim.set_target("actuator", "lever", LEVER_TRAVEL + 0.004)  # latched in HAND
    if STATE["engaged"]:
        lo, hi = STATE["q0"], STATE["q0"] + CLOSED
        stop = hi if q > hi else lo if q < lo else None
        if stop != STATE["stop"]:
            _set_stiffness(sim, "wheel", 0.0 if stop is None else STOP_K)
            if stop is not None:
                sim.set_target("actuator", "wheel", stop)
            STATE["stop"] = stop
    STATE["prev"] = STATE["q"] = q
    sim.set_target("actuator", "pointer", IND_LEN * closed_fraction())
    if TRACE:
        with open(TRACE, "a") as f:
            f.write(json.dumps({
                "t": round(sim.t, 2), "wheel_deg": round(math.degrees(q), 1), "lever_mm": round(lever * 1000, 1),
                "engaged": STATE["engaged"], "closed_pct": round(100 * closed_fraction(), 1),
                "pointer_mm": round(float(s.joints["pointer"]) * 1000, 1), "score": score(None),
            }) + "\n")


# -- scoring ------------------------------------------------------------------------------------------------------------
def closed_fraction() -> float:
    """Valve travel from open (0) to closed (1): wheel turns since the clutch engaged, over one turn."""
    if not STATE["engaged"]:
        return 0.0
    return max(0.0, min(1.0, (STATE["q"] - STATE["q0"]) / CLOSED))


def grade(engaged: bool, fraction: float) -> float:
    if not engaged:
        return 0.0
    if fraction >= 1.0 - SHUT_TOL / CLOSED:
        return 1.0
    return 0.5 if fraction >= 0.5 else 0.25


def success(objects):
    return grade(STATE["engaged"], closed_fraction()) == 1.0


def distance(objects):
    return 1.0 - closed_fraction()


def score(objects):
    """1.0 clutch engaged and valve closed (within 5 degrees of the stop); 0.5 engaged and at least half closed;
    0.25 engaged; 0 otherwise."""
    return grade(STATE["engaged"], closed_fraction())


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Left arm: press the paddle and hold it through the first stroke, then let go and park. Right arm: closed fingers
# go down between two spokes 45 mm from the axis and sweep clockwise, pushing the spoke ahead of them, then lift and
# go back for the next spoke (as in a gate-valve handwheel). AMO_VARIANT selects a shortcut for the checks.
TIP = 0.013
PUSH_R = 0.045
PUSH_Z = HUB_Z - 0.008 + TIP
CLEAR_Z = HUB_Z + 0.006 + 0.020 + TIP
START_DEG, END_DEG = 255.0, 110.0
VARIANTS = {  # name: (lever plan, strokes, direction)
    "demo": ("hold", 6, 1),
    "no_lever": (None, 5, 1),
    "tap_lever": ("tap", 5, 1),
    "engage_only": ("hold", 1, 1),
    "half": ("hold", 3, 1),
    "anticlockwise": ("hold", 4, -1),
}


def _at(deg, z):
    a = math.radians(deg)
    return (VX + PUSH_R * math.cos(a), VY + PUSH_R * math.sin(a), TOP + z)


def demo():
    lever_plan, strokes, direction = VARIANTS[os.environ.get("AMO_VARIANT", "demo")]
    lx, ly = VX + LEVER_AT[0], VY + LEVER_AT[1]
    press_z = TOP + LEVER_TOP - LEVER_TRAVEL - 0.003 + TIP
    hover = TOP + LEVER_TOP + 0.05
    left = [Waypoint(2.0, (lx, ly, hover), FINGER_CLOSED, "above the orange paddle")]
    if lever_plan == "hold":
        left += [Waypoint(1.0, (lx, ly, press_z), FINGER_CLOSED, "press the paddle"),
                 Waypoint(5.0, (lx, ly, press_z), FINGER_CLOSED, "hold it while the wheel turns")]
    elif lever_plan == "tap":
        left += [Waypoint(0.8, (lx, ly, press_z), FINGER_CLOSED, "press the paddle"),
                 Waypoint(0.4, (lx, ly, hover), FINGER_CLOSED, "let it go before turning")]
    left += [Waypoint(1.0, (lx, ly, hover), FINGER_CLOSED, "lift off the paddle"),
             Waypoint(1.5, (0.20, 0.25, hover + 0.05), FINGER_CLOSED, "park")]

    a, b = (START_DEG, END_DEG) if direction > 0 else (END_DEG, START_DEG)
    right = [Waypoint(3.0, _at(a, CLEAR_Z), FINGER_CLOSED, "over the gap between two spokes")]
    for k in range(strokes):
        right.append(Waypoint(0.8, _at(a, PUSH_Z), FINGER_CLOSED, f"stroke {k + 1}: down beside a spoke"))
        for i in range(1, 15):
            right.append(Waypoint(0.2, _at(a + (b - a) * i / 14, PUSH_Z), FINGER_CLOSED, f"stroke {k + 1}: push"))
        right.append(Waypoint(0.6, _at(b, CLEAR_Z), FINGER_CLOSED, f"stroke {k + 1}: lift clear"))
        right.append(Waypoint(1.0, _at(a, CLEAR_Z), FINGER_CLOSED, f"stroke {k + 1}: back"))
    right.append(Waypoint(1.0, (0.20, -0.30, TOP + 0.20), FINGER_CLOSED, "park"))
    return {"left": left, "right": right}

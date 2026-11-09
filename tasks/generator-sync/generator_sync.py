"""On the station table: a light-grey generator control panel in front of the right arm, cabled to a green generator
on a skid (fixed decoration) left of centre. The panel carries a synchroscope (white dial, black pointer, green mark
at 12 o'clock on the side away from the robot), a red CLOSE button and two black speed buttons: RAISE (farther from
the robot) and LOWER (nearer).

The pointer turns at the slip frequency, generator minus grid: clockwise when the generator is fast. It starts at
6 o'clock turning counter-clockwise once every 2 s (generator 0.5 Hz slow). Each RAISE press adds 0.125 Hz, each
LOWER press takes 0.125 Hz away. The first CLOSE press connects the generator: it counts only with the pointer turning
slowly (|slip| <= 0.15 Hz) and near 12 o'clock.

Run it:  inspect-robots run --embodiment yam_arena -E task=generator_sync.py --scorer task_score ...
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import Slider, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HZ = 20.0  # control steps per second

SLIP0_HZ = -0.5  # generator 0.5 Hz slow at the start: one counter-clockwise turn per 2 s
SLIP_STEP_HZ = 0.125  # per RAISE / LOWER press
PHASE0_DEG = 180.0  # pointer starts at 6 o'clock
SLOW_HZ = 0.15  # speed matched: a turn takes 6.7 s or more
FULL_DEG, HALF_DEG = 15.0, 30.0  # pointer within this of 12 o'clock at CLOSE: 1.0 / 0.5
PRESSED = -0.004  # a button counts as pressed 4 mm down (of 8 mm travel)

# The control panel. Part frame origin: the panel's centre; its top face is at z = +0.03.
PX, PY = 0.33, -0.12
GREY, BLACK, WHITE = (0.78, 0.79, 0.76), (0.08, 0.08, 0.08), (0.95, 0.95, 0.95)
DIAL = (0.025, 0.035)  # dial centre in the panel frame (x away from the robot, y to the robot's left)


def _button(name, x, y, size, color):
    return Slider(name, rest=(x, y, 0.03), travel=0.008, rate=200.0, damping=5.0,
                  boxes=(((x, y, 0.036), (size, size, 0.012), color),))


CLOSE = _button("close", -0.045, -0.05, 0.035, (0.80, 0.10, 0.10))
RAISE = _button("raise", 0.06, -0.06, 0.025, BLACK)
LOWER = _button("lower", 0.015, -0.06, 0.025, BLACK)
PANEL = part("panel", pos=(PX, PY, TOP + 0.03), mass=20.0, color=GREY,
             boxes=[((0, 0, 0), (0.20, 0.18, 0.06), GREY),
                    # 12 o'clock mark on the panel just outside the bezel, under the pointer's tip
                    ((DIAL[0] + 0.064, DIAL[1], 0.0305), (0.014, 0.01, 0.001), (0.10, 0.75, 0.20))],
             cylinders=[((*DIAL, 0.031), 0.055, 0.002, BLACK), ((*DIAL, 0.0315), 0.05, 0.003, WHITE)],
             sliders=(CLOSE, RAISE, LOWER))

POINTER_POS = (PX + DIAL[0], PY + DIAL[1], TOP + 0.03 + 0.033 + 0.0015)  # resting on the dial face
POINTER = part("pointer", pos=POINTER_POS, yaw=math.pi, mass=0.02, color=BLACK,  # spawns at 6 o'clock
               boxes=[((0.02, 0, 0), (0.085, 0.008, 0.003), BLACK)])  # origin at the dial centre, tip 6 cm out

# The generator: fixed decoration (boxes without mass don't move), left of centre, out of the arms' parking area.
GX, GY = 0.43, 0.08
GREEN, DARK = (0.12, 0.42, 0.25), (0.25, 0.26, 0.27)
GENERATOR = [
    Box("gen_skid", pos=(GX, GY, TOP + 0.01), size=(0.20, 0.12, 0.02), color=DARK),
    Box("gen_body", pos=(GX, GY, TOP + 0.06), size=(0.14, 0.10, 0.08), color=GREEN),
    *[Box(f"gen_rib{i}", pos=(GX - 0.05 + 0.025 * i, GY, TOP + 0.105), size=(0.008, 0.104, 0.01), color=GREEN)
      for i in range(5)],
    Box("gen_shaft", pos=(GX - 0.085, GY, TOP + 0.06), size=(0.03, 0.025, 0.025), color=DARK),
    Box("gen_terminal", pos=(GX - 0.01, GY - 0.065, TOP + 0.08), size=(0.05, 0.03, 0.04), color=DARK),
    Box("gen_cable", pos=(GX - 0.01, (GY - 0.05 + PY + 0.09) / 2, TOP + 0.005),
        size=(0.012, GY - 0.05 - PY - 0.09, 0.01), color=BLACK),  # under the terminal box to the panel's left edge
]
OBJECTS = [PANEL, POINTER, *GENERATOR]

# The trial record. The module lives across trials, so it is cleared on every reset.
_S = {}


@on_reset
def _reset(environment=None):
    _S.update(slip=SLIP0_HZ, phase=PHASE0_DEG, down={"raise": False, "lower": False}, close=None)


_reset()


def _advance(sliders):
    """One control step of the generator model, given the panel's slider positions (m, negative when pressed)."""
    for name, sign in (("raise", 1), ("lower", -1)):  # a press counts once, on the way down
        down = sliders[name] < PRESSED
        if down and not _S["down"][name]:
            _S["slip"] += sign * SLIP_STEP_HZ
        _S["down"][name] = down
    _S["phase"] = (_S["phase"] + 360.0 * _S["slip"] / HZ + 180.0) % 360.0 - 180.0  # 0 = 12 o'clock
    if _S["close"] is None and sliders["close"] < PRESSED:
        _S["close"] = (_S["slip"], _S["phase"])  # only the first press counts


@on_step
def _step(environment):
    panel = environment.unwrapped.scene[PANEL.name]
    names, q = list(panel.joint_names), panel.data.joint_pos[0]
    _advance({s.name: float(q[names.index(f"{s.name}_slide")]) for s in PANEL.sliders})
    yaw = -math.radians(_S["phase"])  # clockwise seen from above when the generator is fast
    write_pose(environment, POINTER.name, POINTER_POS, (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def score(objects):
    """First CLOSE press: 0 if the pointer was turning faster than SLOW_HZ, else 1.0 within 15 degrees of 12 o'clock,
    0.5 within 30, 0 beyond. Never closed: 0.25 if the speed is matched at the end, else 0."""
    if _S["close"] is None:
        return 0.25 if abs(_S["slip"]) <= SLOW_HZ else 0.0
    slip, phase = _S["close"]
    if abs(slip) > SLOW_HZ:
        return 0.0
    return 1.0 if abs(phase) <= FULL_DEG else 0.5 if abs(phase) <= HALF_DEG else 0.0


def success(objects):
    """Closed in sync; nothing after that can change the score."""
    return _S["close"] is not None and score(objects) == 1.0


def demo():
    """Tap RAISE five times (slip -0.5 -> +0.125 Hz: one slow clockwise turn per 8 s), move over CLOSE, wait, and
    press as the pointer passes 12 o'clock at t = 11.86 s. The press registers 0.2 s after the stroke starts; the wait
    was timed from the printed pointer angles of a scripted run (CLOSE at -2.2 degrees)."""
    top = TOP + 0.072  # button tops
    hover, press = top + 0.015, top - 0.012

    def at(button, z):
        return (PX + button.rest[0], PY + button.rest[1], z)

    steps = [Waypoint(2.0, at(RAISE, hover + 0.05), FINGER_CLOSED, "above RAISE"),
             Waypoint(1.0, at(RAISE, hover), FINGER_CLOSED, "hover RAISE")]
    for i in range(5):
        steps += [Waypoint(0.4, at(RAISE, press), FINGER_CLOSED, f"press RAISE {i + 1}"),
                  Waypoint(0.4, at(RAISE, hover), FINGER_CLOSED, f"release RAISE {i + 1}")]
    steps += [Waypoint(1.5, at(CLOSE, hover), FINGER_CLOSED, "hover CLOSE"),
              Waypoint(3.15, at(CLOSE, hover), FINGER_CLOSED, "wait for 12 o'clock"),
              Waypoint(0.5, at(CLOSE, press), FINGER_CLOSED, "press CLOSE"),
              Waypoint(0.5, at(CLOSE, press), FINGER_CLOSED, "hold"),
              Waypoint(1.5, at(CLOSE, hover + 0.08), FINGER_CLOSED, "lift"),
              Waypoint(1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear")]
    return {"right": steps}

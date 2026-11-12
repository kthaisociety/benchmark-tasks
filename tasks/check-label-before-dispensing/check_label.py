"""On the station table, a pharmacy dispensing bench: four identical white medicine cartons stand in a row in a low
tote, all turned with their label faces away from the arms, so no camera can see a label. Each label carries a
strength band: red (10 mg) on one carton, green (5 mg) on the other three. Which slot holds the 10 mg carton changes
from trial to trial. A dispensing basket stands between the arms, in front of the tote.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import trialkit  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CARD, LABEL, PRINT, RED, GREEN = (0.97, 0.97, 0.96), (0.90, 0.92, 0.94), (0.35, 0.37, 0.40), (0.85, 0.12, 0.12), \
    (0.15, 0.62, 0.30)
TOTE_C, BASKET_C, BENCH = (0.25, 0.45, 0.70), (0.85, 0.85, 0.82), (0.92, 0.92, 0.90)

DEPTH, WIDTH, HEIGHT = 0.04, 0.05, 0.10
ROW_X, FLOOR = 0.33, TOP + 0.004
SLOTS = (0.19, 0.065, -0.065, -0.19)
TEN_MG = "carton_10mg"
CARTONS = {TEN_MG: RED, "carton_5mg_a": GREEN, "carton_5mg_b": GREEN, "carton_5mg_c": GREEN}
START_SLOT = 2  # where the 10 mg carton is declared; each reset moves it to a slot drawn from the trial's seed


def _carton(name, y, band):
    """Label face on the carton's +x side, turned away from the arms at the start."""
    face = DEPTH / 2 + 0.0004
    return trialkit.Labelled(name, pos=(ROW_X, y, FLOOR + HEIGHT / 2), size=(DEPTH, WIDTH, HEIGHT), mass=0.06,
                             color=CARD, panels=(
                                 ((face, 0, 0.004), (0.0008, WIDTH - 0.006, 0.088), LABEL),
                                 ((face + 0.0006, 0, 0.0225), (0.0008, WIDTH - 0.006, 0.05), band),
                                 ((face + 0.0006, 0, -0.018), (0.0008, WIDTH - 0.016, 0.004), PRINT),
                                 ((face + 0.0006, 0, -0.028), (0.0008, WIDTH - 0.024, 0.003), PRINT),
                                 ((-face, 0, -0.03), (0.0008, 0.02, 0.012), PRINT)))


TOTE_X0, TOTE_X1, TOTE_HALF_Y, TOTE_WALL, TOTE_H = 0.27, 0.39, 0.25, 0.005, 0.03
TOTE = [
    Box("tote_floor", pos=((TOTE_X0 + TOTE_X1) / 2, 0, TOP + 0.002), size=(TOTE_X1 - TOTE_X0, 2 * TOTE_HALF_Y, 0.004),
        color=TOTE_C),
    Box("tote_front", pos=(TOTE_X0, 0, TOP + TOTE_H / 2), size=(TOTE_WALL, 2 * TOTE_HALF_Y, TOTE_H), color=TOTE_C),
    Box("tote_back", pos=(TOTE_X1, 0, TOP + TOTE_H / 2), size=(TOTE_WALL, 2 * TOTE_HALF_Y, TOTE_H), color=TOTE_C),
    Box("tote_left", pos=((TOTE_X0 + TOTE_X1) / 2, TOTE_HALF_Y, TOP + TOTE_H / 2),
        size=(TOTE_X1 - TOTE_X0, TOTE_WALL, TOTE_H), color=TOTE_C),
    Box("tote_right", pos=((TOTE_X0 + TOTE_X1) / 2, -TOTE_HALF_Y, TOP + TOTE_H / 2),
        size=(TOTE_X1 - TOTE_X0, TOTE_WALL, TOTE_H), color=TOTE_C),
]

BASKET_XY, BASKET_INSIDE, BASKET_H, BASKET_FLOOR = (0.18, 0.0), (0.15, 0.15), 0.05, TOP + 0.004
_BW, _BO = 0.005, 0.0775  # wall thickness; wall centre from the basket centre
BASKET = [
    Box("basket_floor", pos=(*BASKET_XY, TOP + 0.002), size=(0.16, 0.16, 0.004), color=BASKET_C),
    *[Box(f"basket_wall_{i}", pos=(BASKET_XY[0] + dx, BASKET_XY[1] + dy, TOP + BASKET_H / 2), size=size,
          color=BASKET_C)
      for i, (dx, dy, size) in enumerate([(_BO, 0, (_BW, 0.16, BASKET_H)), (-_BO, 0, (_BW, 0.16, BASKET_H)),
                                          (0, _BO, (0.16, _BW, BASKET_H)), (0, -_BO, (0.16, _BW, BASKET_H))])],
]

DRESSING = [
    Box("bench_mat", pos=(0.34, 0.0, TOP + 0.0005), size=(0.30, 0.62, 0.001), color=BENCH),
    Box("prescription", pos=(0.18, 0.16, TOP + 0.0012), size=(0.10, 0.07, 0.001), color=(0.98, 0.98, 0.95)),
    Box("prescription_lines", pos=(0.18, 0.16, TOP + 0.0018), size=(0.06, 0.04, 0.0004), color=PRINT),
    Box("counting_tray", pos=(0.58, 0.20, TOP + 0.006), size=(0.14, 0.08, 0.012), color=(0.20, 0.55, 0.75)),
    Box("pharmacy_bag", pos=(0.60, -0.22, TOP + 0.05), size=(0.09, 0.05, 0.10), color=(0.95, 0.95, 0.92)),
    Cylinder("tablet_bottle", pos=(0.58, -0.02, TOP + 0.04), radius=0.025, height=0.08, color=(0.65, 0.40, 0.15)),
]

OBJECTS = [*TOTE, *BASKET, *DRESSING,
           *[_carton(n, SLOTS[k], band) for (n, band), k in zip(CARTONS.items(), (START_SLOT, 0, 1, 3))]]

SHOWN_PIXELS, SHOWN_STEPS = 15, 10  # red band pixels in the 640 x 360 top camera image, for 0.5 s at 20 Hz
_S = {"seen": 0, "shown": False, "slot": START_SLOT}


@trialkit.on_reset
def _new_trial(environment, rng):
    """Put the 10 mg carton in a slot drawn from the trial's seed (or LABEL_SLOT) and the others in the rest."""
    forced = os.environ.get("LABEL_SLOT")
    slot = int(forced) if forced is not None else int(rng.integers(len(SLOTS)))
    others = [k for k in range(len(SLOTS)) if k != slot]
    for name, k in zip(CARTONS, (slot, *others)):
        trialkit.place(environment, name, (ROW_X, SLOTS[k], FLOOR + HEIGHT / 2))
    _S.update(seen=0, shown=False, slot=slot)


def _red_mask(image):
    rgb = image.astype(int)
    return (rgb[..., 0] > 140) & (rgb[..., 1] < 70) & (rgb[..., 2] < 70)


def _inside_basket(state):
    dx, dy = np.abs(state.pos[:2] - BASKET_XY)
    return bool(dx < BASKET_INSIDE[0] / 2 and dy < BASKET_INSIDE[1] / 2)


def _dispensed(state):
    """Anywhere inside the basket walls, held or not: what counts against a wrong carton."""
    return _inside_basket(state) and state.low[2] < TOP + BASKET_H + 0.02


def _resting_in_basket(state):
    """Released and at rest on the basket floor, with no closed gripper within 8 cm of it."""
    held = any(opening < 0.5 and np.linalg.norm(tcp - state.pos) < 0.08
               for tcp, opening in trialkit.grippers().values())
    return _inside_basket(state) and state.low[2] < BASKET_FLOOR + 0.01 and state.resting() and not held


def score(objects):
    """0 if a 5 mg carton is in the basket. Otherwise 1.0 for the 10 mg carton at rest in the basket, released, after
    its red band had been visible in the top camera image for 0.5 s; 0.25 for that band shown without the carton
    ending at rest in the basket; 0 otherwise."""
    ten = objects[TEN_MG]
    image = trialkit.top_image()
    if image is not None and not _dispensed(ten):
        _S["seen"] = _S["seen"] + 1 if np.count_nonzero(_red_mask(image)) >= SHOWN_PIXELS else 0
        _S["shown"] = _S["shown"] or _S["seen"] >= SHOWN_STEPS
    if os.environ.get("LABEL_DEBUG"):
        red = np.count_nonzero(_red_mask(image)) if image is not None else -1
        print(f"LABEL slot={_S['slot']} red={red} shown={_S['shown']} ten={np.round(ten.pos, 3)} "
              f"rest={_resting_in_basket(ten)}", flush=True)
    if any(_dispensed(objects[c]) for c in CARTONS if c != TEN_MG):
        return 0.0
    if not _S["shown"]:
        return 0.0
    return 1.0 if _resting_in_basket(ten) else 0.25


# -- scripted demo -------------------------------------------------------------------------------------------------

GRIP = 0.07 + 0.008
LIFT = 0.13
CARRY = TOP + 0.17
CAM = station.top_camera()
PARK = {"left": (0.12, 0.42, TOP + 0.20), "right": (0.12, -0.42, TOP + 0.20)}


def _quarter(y):
    grip, lifted = FLOOR + GRIP, FLOOR + LIFT
    return [
        (1.5, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "above", 0.0),
        (1.5, (ROW_X, y, grip), FINGER_OPEN, "descend", 0.0),
        (0.6, (ROW_X, y, grip), FINGER_OPEN, "settle", 0.0),
        (0.8, (ROW_X, y, grip), FINGER_CLOSED, "close", 0.0),
        (1.0, (ROW_X, y, lifted), FINGER_CLOSED, "lift clear of the tote", 0.0),
        (1.5, (ROW_X, y, lifted), FINGER_CLOSED, "turn a quarter", math.pi / 2),
        (1.2, (ROW_X, y, grip + 0.004), FINGER_CLOSED, "lower", math.pi / 2),
        (0.6, (ROW_X, y, grip + 0.004), FINGER_OPEN, "release", math.pi / 2),
    ]


def _turn(y):
    """Lift a carton clear of the tote and turn it a half turn, in two quarter turns with a fresh grip between, so its
    label faces the arms; set it back down."""
    steps = [*_quarter(y), *_quarter(y), (1.0, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "retreat", math.pi / 2)]
    return [Waypoint(*s) for s in steps]


def _image_u(point, width):
    """Column at which the top camera sees a point, in an image ``width`` pixels wide."""
    cam = quat_xyzw_to_matrix(np.asarray(CAM.rot_xyzw)).T @ (np.asarray(point) - np.asarray(CAM.pos))
    return (CAM.lens.cx + CAM.lens.fx * cam[0] / -cam[2]) * width / CAM.lens.width


def _red_slot(image):
    """The slot under the red band the top camera sees, or None."""
    cols = np.nonzero(_red_mask(image))[1]
    if len(cols) < 2:
        return None
    band = FLOOR + HEIGHT / 2 + 0.0225
    us = [_image_u((ROW_X - DEPTH / 2, y, band), image.shape[1]) for y in SLOTS]
    return int(np.argmin([abs(u - np.median(cols)) for u in us]))


def _dispense(y, park):
    s = [
        (1.5, (ROW_X, y, TOP + 0.20), FINGER_OPEN, "above the red-band carton", 0.0),
        (1.5, (ROW_X, y, FLOOR + GRIP), FINGER_OPEN, "descend", 0.0),
        (0.6, (ROW_X, y, FLOOR + GRIP), FINGER_OPEN, "settle", 0.0),
        (0.8, (ROW_X, y, FLOOR + GRIP), FINGER_CLOSED, "close", 0.0),
        (1.0, (ROW_X, y, CARRY), FINGER_CLOSED, "lift", 0.0),
        (2.0, (*BASKET_XY, CARRY), FINGER_CLOSED, "carry to the basket", 0.0),
        (1.5, (*BASKET_XY, BASKET_FLOOR + GRIP + 0.002), FINGER_CLOSED, "lower into the basket", 0.0),
        (0.8, (*BASKET_XY, BASKET_FLOOR + GRIP + 0.002), FINGER_OPEN, "release", 0.0),
        (1.0, (*BASKET_XY, CARRY), FINGER_OPEN, "retreat", 0.0),
        (1.5, park, FINGER_OPEN, "move clear", 0.0),
    ]
    return [Waypoint(*w) for w in s]


def _variant(mode, slot):
    """Scripted wrong strategies for the README's checks (LABEL_VARIANT, with LABEL_SLOT set)."""
    other = {0: 1, 1: 0, 2: 3, 3: 2}[slot]
    pick = {"guess_right": slot, "guess_wrong": other}.get(mode)
    if pick is None:
        return {"left": [Waypoint(1.0, PARK["left"], FINGER_OPEN, "stay clear")]}
    arm = "left" if SLOTS[pick] > 0 else "right"
    return {arm: _dispense(SLOTS[pick], PARK[arm])}


def demo_stages():
    """Each arm turns its two cartons so their labels face the arms; the top camera then shows one red band, and the
    arm on that side lifts that carton into the basket. The green-band cartons stay in the tote."""
    mode = os.environ.get("LABEL_VARIANT")
    if mode in ("none", "guess_right", "guess_wrong"):
        yield _variant(mode, int(os.environ["LABEL_SLOT"]))
        return
    turn_time = sum(w.duration for w in _turn(SLOTS[2]))
    left = [*_turn(SLOTS[0]), *_turn(SLOTS[1]), Waypoint(1.5, PARK["left"], FINGER_OPEN, "move clear", math.pi / 2)]
    right = [*_turn(SLOTS[3]), Waypoint(turn_time, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "wait", math.pi / 2),
             *_turn(SLOTS[2]), Waypoint(1.5, PARK["right"], FINGER_OPEN, "move clear", math.pi / 2)]
    view = yield {"left": left, "right": right}
    slot = _red_slot(view.images["top_cam"])
    if os.environ.get("LABEL_DEBUG"):
        print(f"LABEL demo sees the red band at slot {slot}", flush=True)
    if mode == "turn_wrong" and slot is not None:
        slot = {0: 1, 1: 0, 2: 3, 3: 2}[slot]
    if slot is not None and mode != "turn_only":
        arm = "left" if SLOTS[slot] > 0 else "right"
        yield {arm: _dispense(SLOTS[slot], PARK[arm])}


demo = trialkit.staged(demo_stages)

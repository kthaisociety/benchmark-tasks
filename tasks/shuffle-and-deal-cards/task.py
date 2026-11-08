"""A card table at the bimanual station. Six thick card tiles, two red, two green and two blue, stand in one deck
on the dark grey mat near the left arm, sorted by colour: red at the bottom, then green, then blue on top. Two
light grey mats, one in front of the deck and one at the middle of the near edge, are free for shuffling. Three
yellow seat mats across the far side of the table are the players' places: left, middle and right.
"""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import scoring  # noqa: E402
from partkit import on_reset  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "Shuffle the deck on the dark mat so that no two cards of the same colour touch, using the grey mats, then "
    "deal it from the top, one card at a time, clockwise to the three yellow seats, starting with the seat on "
    "your left, until each seat has two cards."
)

CARD = (0.090, 0.056, 0.016)  # x, y, z (m); the grippers close across the 56 mm side
CARD_MASS = 0.040
MAT_Z = 0.002
PILE_MAT = (0.11, 0.08, MAT_Z)
SEAT_MAT = (0.13, 0.10, MAT_Z)
DECK = (0.22, 0.13)
SPARE_SIDE = (0.34, 0.13)
SPARE_MIDDLE = (0.22, 0.00)
SEAT_LEFT = (0.36, 0.25)
SEAT_MIDDLE = (0.40, 0.00)
SEAT_RIGHT = (0.36, -0.25)

COLOURS = {"red": (0.84, 0.14, 0.14), "green": (0.13, 0.62, 0.27), "blue": (0.13, 0.33, 0.84)}
START = ["red", "red", "green", "green", "blue", "blue"]  # bottom to top
CARDS = [f"card_{i}" for i in range(len(START))]
CARD_COLOUR = dict(zip(CARDS, START))

PILES = (
    scoring.Zone("deck_mat", DECK, (0.03, 0.025)),
    scoring.Zone("grey_side", SPARE_SIDE, (0.03, 0.025)),
    scoring.Zone("grey_middle", SPARE_MIDDLE, (0.03, 0.025)),
)
SEATS = (
    scoring.Zone("seat_left", SEAT_LEFT, (0.04, 0.035)),
    scoring.Zone("seat_middle", SEAT_MIDDLE, (0.04, 0.035)),
    scoring.Zone("seat_right", SEAT_RIGHT, (0.04, 0.035)),
)
SPAWN_GAP = 0.001


def card_z(level):
    """Centre height of a card at ``level`` (0 = on the mat) in a settled stack."""
    return TOP + MAT_Z + CARD[2] / 2 + level * CARD[2]


OBJECTS = [
    Box("deck_mat", pos=(*DECK, TOP + MAT_Z / 2), size=PILE_MAT, color=(0.22, 0.23, 0.25)),
    Box("grey_side", pos=(*SPARE_SIDE, TOP + MAT_Z / 2), size=PILE_MAT, color=(0.66, 0.67, 0.69)),
    Box("grey_middle", pos=(*SPARE_MIDDLE, TOP + MAT_Z / 2), size=PILE_MAT, color=(0.66, 0.67, 0.69)),
    *[Box(s.name, pos=(*s.center, TOP + MAT_Z / 2), size=SEAT_MAT, color=(0.96, 0.80, 0.16)) for s in SEATS],
    *[
        Box(name, pos=(*DECK, card_z(level) + SPAWN_GAP * (level + 1)), size=CARD,
            color=COLOURS[CARD_COLOUR[name]], mass=CARD_MASS)
        for level, name in enumerate(CARDS)
    ],
]

TRACKER = scoring.Tracker(colors=CARD_COLOUR, piles=PILES, seats=SEATS, table_top=TOP)
_DEBUG = {"on": bool(os.environ.get("CARD_DEBUG")), "step": 0}


@on_reset
def _reset(environment=None):
    TRACKER.reset()
    _DEBUG["step"] = 0


def _cards(objects):
    return {
        name: scoring.Card(
            pos=np.asarray(objects[name].pos, dtype=float),
            up=scoring.up_component(objects[name].quat_xyzw),
            speed=float(np.linalg.norm(objects[name].lin_vel)),
        )
        for name in CARDS
    }


def score(objects):
    cards = _cards(objects)
    value = TRACKER.update(cards)
    _DEBUG["step"] += 1
    if _DEBUG["on"] and _DEBUG["step"] % 40 == 0:
        where = " ".join(
            f"{n[-1]}:{TRACKER.zone_of(c) or '-'}@{(c.pos[2] - TOP) * 1000:.0f}" for n, c in cards.items()
        )
        print(f"[cards] step {_DEBUG['step']} score {value} deck {TRACKER.deck} dealt {TRACKER.deals} "
              f"misdeal {TRACKER.misdeal} | {where}", flush=True)
    return value


SUCCESS_HOLD_S = 3.0  # the arms have moved clear of the seats before the trial ends


def success(objects):
    return TRACKER.score(_cards(objects)) == 1.0


# -- scripted demo -------------------------------------------------------------------------------------------------

GRIP_ABOVE_CENTRE = 0.008  # TCP height above the card centre for a mid-height grip
PLACE_CLEARANCE = 0.002
TRAVEL = TOP + 0.13
SPEED = 1.3
PARK = {"left": (0.14, 0.40, TOP + 0.16), "right": (0.14, -0.40, TOP + 0.16)}


def _hold(arm, seconds, label="wait"):
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label)


def demo():
    """Restack the deck as blue, green, red, green, blue, red (bottom to top) on the middle grey mat, then deal."""
    piles = {"deck_mat": list(CARDS), "grey_side": [], "grey_middle": [], **{s.name: [] for s in SEATS}}
    where = {z.name: z.center for z in (*PILES, *SEATS)}
    plan = {"left": [_hold("left", 1.5, "park")], "right": [_hold("right", 1.5, "park")]}

    def move(arm, source, target):
        card = piles[source].pop()
        pick = (*where[source], card_z(len(piles[source])) + GRIP_ABOVE_CENTRE)
        place = (*where[target], card_z(len(piles[target])) + GRIP_ABOVE_CENTRE + PLACE_CLEARANCE)
        piles[target].append(card)
        steps = pick_place_waypoints(pick, place, travel_z=TRAVEL, speed=SPEED)
        steps.append(Waypoint(1.0, PARK[arm], FINGER_OPEN, "park"))
        other = "left" if arm == "right" else "right"
        plan[arm].extend(steps)
        plan[other].append(_hold(other, sum(w.duration for w in steps)))

    for source, target in (
        ("deck_mat", "grey_middle"),  # blue
        ("deck_mat", "grey_side"),  # blue
        ("deck_mat", "grey_middle"),  # green
        ("deck_mat", "grey_side"),  # green
        ("deck_mat", "grey_middle"),  # red
        ("grey_side", "grey_middle"),  # green
        ("grey_side", "grey_middle"),  # blue
        ("deck_mat", "grey_middle"),  # red
    ):
        move("left", source, target)
    for seat in [s.name for s in SEATS] * 2:
        move("right" if seat == "seat_right" else "left", "grey_middle", seat)
    for arm in plan:
        plan[arm].append(_hold(arm, 2.0, "stay clear of the camera"))
    return plan

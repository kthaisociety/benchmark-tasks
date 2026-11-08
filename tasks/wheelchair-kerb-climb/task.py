"""A 1:4 attendant wheelchair stands on a dark road surface facing a grey kerb 15 mm high, about 1 cm from its
face. A dark grey weighted dummy sits on the blue seat. The small front casters are too small to roll up the
kerb; the large rear wheels can. At the back, low down, an orange tipping bar joins the ends of the chair's two
tipping levers: pressing it down tips the chair back onto its rear wheels. A black push bar joins the ends of the
two push handles at the top of the backrest.

Run with run.py (it builds the chair as a jointed body): see README.md.
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chair as C  # noqa: E402
import layout as L  # noqa: E402
import scoring  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
INSTRUCTION = ("Get the wheelchair up onto the grey kerb so that all four of its wheels rest on top, without tipping "
               "it over or letting the dummy fall off the seat. Then let go of the chair.")
ROAD, KERB_GREY, DUMMY_GREY = (0.16, 0.16, 0.17), (0.66, 0.66, 0.64), (0.30, 0.31, 0.33)
PAINT = 0.0004
D = L.DUMMY

OBJECTS = [
    Box("chair", pos=(*L.AXLE0, TOP + 0.0005), size=(0.01, 0.01, 0.01), mass=0.5),  # replaced by chair.build() in run.py
    Box("dummy", pos=(L.AXLE0[0] + D["x"], 0.0, TOP + C.SEAT["top"] + D["h"] / 2 + 0.001), size=(D["w"], D["d"], D["h"]),
        mass=D["mass"], color=DUMMY_GREY),
    Box("kerb", pos=((L.KERB_X0 + L.KERB["x1"]) / 2, 0.0, TOP + L.KERB_H / 2), size=(L.KERB["x1"] - L.KERB_X0, 2 * L.KERB["half_y"], L.KERB_H),
        color=KERB_GREY),
    Box("road", pos=((L.ROAD["x0"] + L.KERB_X0) / 2, 0.0, TOP + PAINT / 2), size=(L.KERB_X0 - L.ROAD["x0"], 2 * L.ROAD["half_y"], PAINT), color=ROAD),
]

_trial = {"t": scoring.Trial(), "last": 0.0}


def reset_trial():
    _trial["t"], _trial["last"] = scoring.Trial(), 0.0


def score(objects):
    _trial["last"] = _trial["t"].step(objects)
    return _trial["last"]


def success(objects):
    return _trial["last"] >= 1.0


def fault():
    return _trial["t"].fault


# -- the scripted demo: one arm; the other stays parked ---------------------------------------------------------------
VARIANT = os.environ.get("CHAIR_VARIANT", "demo")  # demo, or a shortcut for the checks in README.md


def _tcp_on(z_surface, press=0.0):
    """TCP height whose closed pad tips sit at z_surface, pressed `press` below it."""
    return z_surface + PAD_TIP_PAST_TCP - press


def demo():
    ax, _ = L.AXLE0
    hover = TOP + 0.20
    gc = FINGER_CLOSED
    park = [Waypoint(2.0, (0.30, 0.27, hover), gc, "park")]
    lx, ly, lz = ax + C.LEVER_TOP[0], C.LEVER_TOP[1], TOP + C.LEVER_TOP[2]
    bz = TOP + C.BAR_Z + PAD_TIP_PAST_TCP
    b0 = ax + C.BAR_BACK - 0.012  # just behind the push bar at the start
    if VARIANT == "nothing":
        return {"left": park, "right": [Waypoint(2.0, (0.30, -0.27, hover), gc, "park")]}
    if VARIANT == "shove":  # push straight at the kerb without tipping
        return {"left": park, "right": [Waypoint(2.0, (b0 - 0.03, 0.0, hover), gc, "hover"),
                                        Waypoint(1.5, (b0 - 0.03, 0.0, bz), gc, "behind push bar"),
                                        Waypoint(7.0, (b0 + 0.20, 0.0, bz), gc, "shove"),
                                        Waypoint(1.5, (0.20, -0.10, TOP + 0.25), gc, "clear")]}
    tip = 0.050 if VARIANT == "overtip" else 0.022
    roll = 0.07
    qx = ax + roll + C.BAR_BACK - 0.012  # just behind the push bar once rolled
    right = [Waypoint(2.0, (lx, ly, hover), gc, "over tipping bar"),
             Waypoint(1.5, (lx, ly, _tcp_on(lz) + 0.004), gc, "down to bar"),
             Waypoint(2.0, (lx, ly, _tcp_on(lz, tip)), gc, "press: tip back"),
             Waypoint(1.0, (lx, ly, _tcp_on(lz, tip)), gc, "hold"),
             Waypoint(3.5, (lx + roll, ly, _tcp_on(lz, tip)), gc, "roll tipped"),
             Waypoint(2.0, (lx + roll, ly, _tcp_on(lz) + 0.03), gc, "let bar up: casters on the kerb")]
    if VARIANT == "casters":  # stop with only the casters up
        return {"left": park, "right": right + [Waypoint(1.5, (lx + roll - 0.02, ly, hover), gc, "up"),
                                                Waypoint(1.5, (0.20, -0.10, TOP + 0.25), gc, "clear")]}
    right += [Waypoint(1.0, (lx + roll - 0.02, ly, _tcp_on(lz) + 0.03), gc, "back off"),
              Waypoint(1.5, (lx + roll - 0.02, ly, hover), gc, "up"),
              Waypoint(1.5, (qx - 0.03, ly, bz), gc, "behind push bar"),
              Waypoint(5.5, (qx + 0.15, ly, bz), gc, "push: rear wheels up"),
              Waypoint(1.0, (qx + 0.12, ly, bz), gc, "back off"),
              Waypoint(1.5, (qx + 0.12, ly, hover), gc, "up"),
              Waypoint(1.5, (0.20, -0.10, TOP + 0.25), gc, "clear")]
    return {"left": park, "right": right}

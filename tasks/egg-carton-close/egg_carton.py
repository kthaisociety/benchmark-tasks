"""A pulp dozen-egg carton is closed after eggs were taken out. It sits on the counter open: the lid stands back past
upright, sprung open by the pulp hinge, and the front locking flap hangs outward. The lid only stays shut when the
flap is tucked up inside its front skirt: a bead along the flap's top then sits above a bead inside the skirt, and the
flap's own spring keeps it there. Shut with the flap left outside, the lid springs back open. Let go of a tucked flap
and it creeps back out within about a second and a half (a soft pulp hinge), so the flap has to be held in until the
lid is nearly down, then the lid brought the last few centimetres down over it.
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import gripkit  # noqa: E402
from hingekit import Hinge, hinged  # noqa: E402
from partkit import env, on_reset, on_step  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Close the egg carton."

PULP, EGG = (0.70, 0.66, 0.58), (0.93, 0.86, 0.74)
CX = float(os.environ.get("EGG_CX", 0.25))   # carton centre, m in front of the arm bases
HB = 0.045            # base (cup tray) height
BX = 0.05             # base half depth (x): 10 cm deep, 30 cm long (y)
BY = 0.15
OPEN = float(os.environ.get("EGG_OPEN", 100.0))      # lid drawn open this far (deg); 0 on its joint
REST = float(os.environ.get("EGG_REST", 60.0))       # flap drawn hanging this far outward (deg)
IN_MAX = 18.0         # flap stops against the front row of eggs this far inward of upright (deg)
LID_RATE = float(os.environ.get("EGG_LID_RATE", 0.02))     # N m/rad: about 0.35 N at the lid's front when shut
FLAP_RATE = float(os.environ.get("EGG_FLAP_RATE", 0.0086))  # N m/rad: about 0.3 N at the flap's top when upright
FLAP_DAMP = float(os.environ.get("EGG_FLAP_DAMP", 0.10))    # N m s/rad: a tucked flap creeps out in ~1.5 s

LP = (BX, 0.0, HB)                 # lid hinge line (back top edge of the base), part frame
FP = (-BX - 0.001, 0.0, HB)        # flap hinge line (front top edge of the base)


def _rot(box, pivot, deg):
    """A box drawn in the closed position, turned deg about the y axis through pivot (USD/joint sense)."""
    (cx, cy, cz), size, color = box[:3]
    a0 = box[3] if len(box) > 3 else 0.0
    a = math.radians(deg)
    x, z = cx - pivot[0], cz - pivot[2]
    xr, zr = x * math.cos(a) + z * math.sin(a), -x * math.sin(a) + z * math.cos(a)
    t = math.radians(deg + a0) / 2
    return ((pivot[0] + xr, cy, pivot[2] + zr), size, color, (math.cos(t), 0.0, math.sin(t), 0.0))


def _eggs(xr):
    return [((xr, -0.125 + 0.05 * k, HB), 0.021, 0.04, EGG) for k in range(6)]


# lid, closed: domed top over the eggs, side walls outside the base's ends, front skirt with an inward bead
_LID = [((-0.00425, 0.0, 0.0815), (0.1085, 0.308, 0.003), PULP),          # top
        ((-0.057, 0.0, 0.0705), (0.003, 0.308, 0.025), PULP),             # front skirt, z 5.8-8.3 cm
        ((-0.054, 0.0, 0.062), (0.003, 0.300, 0.004), PULP),              # bead inside the skirt, z 6.0-6.4 cm
        ((-0.004, 0.1525, 0.0665), (0.109, 0.003, 0.033), PULP),          # end walls
        ((-0.004, -0.1525, 0.0665), (0.109, 0.003, 0.033), PULP),
        ((0.049, 0.0, 0.064), (0.003, 0.308, 0.038), PULP)]               # back wall at the hinge
# flap, upright: a 3 cm leaf with a bead along its top, flat below and sloped above (the skirt's bead cams it in)
_FLAP = [((-0.051, 0.0, 0.060), (0.002, 0.290, 0.030), PULP),
         ((-0.05325, 0.0, 0.0695), (0.0025, 0.290, 0.003), PULP),
         ((-0.0530, 0.0, 0.0722), (0.0035, 0.290, 0.0010), PULP, -45.0)]

BACK = hinged("carton_back", pos=(CX, 0.0, TOP), mass=0.375, friction=1.2, color=PULP,
              boxes=[((BX / 2 + 0.0005, 0.0, HB / 2), (BX - 0.001, 2 * BY, HB), PULP)],
              cylinders=_eggs(0.025),
              hinges=[Hinge("lid", pivot=LP, axis="Y", lower=-OPEN - 1.5, upper=0.5, rate=LID_RATE,
                            damping=0.002, mass=0.02, boxes=[_rot(b, LP, OPEN) for b in _LID])])
FRONT = hinged("carton_front", pos=(CX, 0.0, TOP), mass=0.375, friction=1.2, color=PULP,
               boxes=[((-BX / 2 - 0.0005, 0.0, HB / 2), (BX - 0.001, 2 * BY, HB), PULP)],
               cylinders=_eggs(-0.025),
               hinges=[Hinge("flap", pivot=FP, axis="Y", lower=-5.0, upper=REST + IN_MAX, rate=FLAP_RATE,
                             damping=FLAP_DAMP, mass=0.005, friction=0.5, boxes=[_rot(b, FP, -REST) for b in _FLAP])])
OBJECTS = [BACK, FRONT]

_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.update(step=0, scored=None, last=0.0, recent=[], start=None)


@on_step
def _count(environment=None):
    _S["step"] += 1


_reset()


def _tips():
    """Both arms' finger-pad tips (m, env frame) and finger opening fractions."""
    e = env()
    if e is None:
        return []
    scene = e.unwrapped.scene
    org = np.asarray(getattr(scene.env_origins, "torch", scene.env_origins)[0].detach().cpu(), float)
    out = []
    for name in ("left_robot", "right_robot"):
        arm = scene[name]
        tips = getattr(arm.data.body_pos_w, "torch", arm.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        out += list(tips[arm.find_bodies(["tip_left", "tip_right"], preserve_order=True)[0]] - org)
    return out


def _clear() -> bool:
    """Every finger tip at least 3 cm outside the carton's closed outline (above, in front, behind or beyond)."""
    for t in _tips():
        dx = max(abs(t[0] - CX) - (BX + 0.009), 0.0)
        dy = max(abs(t[1]) - (BY + 0.005), 0.0)
        dz = max(t[2] - TOP - 0.085, 0.0)
        if math.sqrt(dx * dx + dy * dy + dz * dz) < 0.03:
            return False
    return True


def _state(objects):
    lid = objects["carton_back"].sliders["lid"] + OPEN          # 0 = shut, OPEN = as found
    flap = objects["carton_front"].sliders["flap"] - REST       # 0 = upright, + = tucked in, -REST = as found
    return lid, flap


def score(objects) -> float:
    """End state only. 1.0: the lid within 5 degrees of shut with the flap within 6 degrees of upright (inside the
    skirt, where its bead holds the lid), both carton halves within 1 cm of where they started, no finger tip within
    3 cm of the carton, and the lid still (moved under 0.5 degrees) for the last second. 0.5: the same, but the carton
    was pushed more than 1 cm. Otherwise 0: a lid shut over a flap left outside springs back open."""
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    lid, flap = _state(objects)
    pb, pf = np.asarray(objects["carton_back"].pos, float), np.asarray(objects["carton_front"].pos, float)
    if _S["start"] is None:
        _S["start"] = (pb.copy(), pf.copy())
    moved = max(np.linalg.norm((pb - _S["start"][0])[:2]), np.linalg.norm((pf - _S["start"][1])[:2]))
    r = _S["recent"]
    r.append((lid, _clear()))
    del r[:-20]
    still = len(r) == 20 and all(abs(a - lid) < 0.5 and c for a, c in r)
    locked = lid <= 5.0 and flap >= -6.0
    s = (1.0 if moved <= 0.01 else 0.5) if (locked and still) else 0.0
    _S["last"] = s
    if os.environ.get("EGG_DEBUG") and _S["step"] % 10 == 0:
        t = _tips()
        tt = " ".join(f"{np.round((p - (CX, 0, TOP)) * 1000, 0).tolist()}" for p in t[::2])
        print(f"[egg] step {_S['step']} lid {lid:.1f} flap {flap:.1f} moved {moved * 1000:.1f}mm clear {r[-1][1]} "
              f"score {s:.2f} | tips {tt}", flush=True)
    return s


# -- scripted demo: the left arm lifts the flap from under its outer face and tucks it in against the eggs; the right
# arm comes down behind the lid, pushes it over and down to 30 degrees open and waits; the left arm lets the flap go
# and backs out; the right arm brings the lid down over the flap at once, pauses, and lifts away. ---------------------
FZ = float(os.environ.get("EGG_FZ", 0.012))   # TCP height above the point of the finger that pushes
FT = float(os.environ.get("EGG_FT", 0.011))
LT = float(os.environ.get("EGG_LT", 0.006))   # lid: TCP this far off the lid face
LZ = float(os.environ.get("EGG_LZ", 0.028))   # the finger reaches this far below the TCP: lift lid targets by it   # finger half thickness: TCP this far off the pushed face
YL, YR = 0.08, -0.08


def _lid_pt(c, d, h):
    """World point on the lid's top surface, d from its hinge, h off it, with the lid c degrees open."""
    v = (-d, 0.083 - HB + h)
    a = math.radians(c)
    return (CX + LP[0] + v[0] * math.cos(a) + v[1] * math.sin(a), TOP + LP[2] - v[0] * math.sin(a) + v[1] * math.cos(a))


def _flap_pt(u, r, h):
    """World point on the flap's outer face, r up from its hinge, h out from it, the flap u degrees in of upright."""
    a = math.radians(u)
    return (CX + FP[0] - h * math.cos(a) + r * math.sin(a), TOP + FP[2] + h * math.sin(a) + r * math.cos(a))


def demo():
    v = os.environ.get("EGG_VARIANT", "")
    R = gripkit.pitched(0.0)
    C = FINGER_CLOSED
    hold_c = float(os.environ.get("EGG_HOLD", 45.0))
    tuck = float(os.environ.get("EGG_TUCK", 40.0))
    rf, d = float(os.environ.get("EGG_RF", 0.025)), float(os.environ.get("EGG_D", 0.095))

    left = [Waypoint(2.0, (CX - 0.13, YL, TOP + 0.15), C, "above, in front of the flap", R)]
    x, z = _flap_pt(-REST, rf, FT + 0.012)
    left += [Waypoint(1.5, (x - 0.01, YL, z - 0.006 + FZ), C, "outside the flap, low", R)]
    for u in np.linspace(-REST, tuck, 9):
        x, z = _flap_pt(u, rf, FT)
        left.append(Waypoint(0.3, (x, YL, z + FZ), C, f"tuck {u:.0f}", R))

    right = [Waypoint(2.0, (CX + 0.12, YR, TOP + 0.17), C, "above, behind the lid", R)]
    x, z = _lid_pt(OPEN, d, LT + 0.015)
    right += [Waypoint(1.5, (x, YR, z), C, "behind the lid", R)]
    for c in np.linspace(OPEN, hold_c, 9):
        x, z = _lid_pt(c, d, LT)
        right.append(Waypoint(0.35, (x, YR, z + LZ * (1 - c / OPEN)), C, f"lid {c:.0f}", R))

    t_l = sum(w.duration for w in left)
    t_r = sum(w.duration for w in right)
    t_go = max(t_l, t_r) + 0.8
    lp, rp = left[-1].pos, right[-1].pos
    left.append(Waypoint(t_go - t_l, lp, C, "hold the flap in", R))
    right.append(Waypoint(t_go - t_r, rp, C, "hold the lid", R))
    if v == "lid_only":          # wrong: shut the lid over the flap left outside
        left = left[:1] + [Waypoint(t_go - left[0].duration, left[0].pos, C, "wait", R)]
    if v == "slow":              # wrong: tuck the flap, let go, then close the lid 2.5 s later
        right[-1] = Waypoint(t_go - t_r + 2.5, rp, C, "hold the lid", R)
    left += [Waypoint(0.4, lp, C, "keep it in while the lid comes down to 20 degrees", R),
             Waypoint(0.3, (lp[0] - 0.04, YL, lp[2] + 0.005), C, "let go of the flap, back out", R),
             Waypoint(1.5, (CX - 0.15, YL, TOP + 0.18), C, "clear", R)]
    shut = []
    mid = float(os.environ.get("EGG_MID", 20.0))
    x, z = _lid_pt(mid, d, LT)
    shut.append(Waypoint(0.4, (x, YR, z + LZ * (1 - mid / OPEN)), C, f"lid down to {mid:.0f}", R))
    for c in np.linspace(mid, float(os.environ.get("EGG_SHUT", -16.0)), 4)[1:]:
        x, z = _lid_pt(c, d, LT)
        shut.append(Waypoint(0.15, (x, YR, z + LZ * (1 - c / OPEN)), C, f"shut {c:.0f}", R))
    right += shut + [Waypoint(0.8, right[-1].pos if not shut else shut[-1].pos, C, "press shut", R)]
    x, z = shut[-1].pos[0], shut[-1].pos[2]
    right += [Waypoint(1.2, (x, YR, z + 0.10), C, "lift away", R),
              Waypoint(1.5, (CX, YR - 0.08, TOP + 0.20), C, "clear", R)]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    t_end = max(t_l, t_r) + 2.0
    left.append(Waypoint(t_end - t_l, left[-1].pos, C, "wait", R))
    right.append(Waypoint(t_end - t_r, right[-1].pos, C, "wait", R))
    if v == "nothing":
        return {"left": [Waypoint(3.0, (0.18, 0.30, TOP + 0.22), FINGER_OPEN, "park")],
                "right": [Waypoint(3.0, (0.18, -0.30, TOP + 0.22), FINGER_OPEN, "park")]}
    return {"left": left, "right": right}

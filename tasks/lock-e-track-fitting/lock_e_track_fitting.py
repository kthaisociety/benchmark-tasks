"""Van floor: lock a spring E-track tie-off fitting into a slot of a section of floor E-track. The track's 2.7 mm
steel face stands 10 mm off a plywood floor panel, with 62.2 x 14.7 mm slots on 50.8 mm centres. The fitting's head
spans 70 mm across its two lips with the spring latch out, so it only goes down a slot with the latch squeezed in
10 mm (60 mm, 1.1 mm spare each end). Once the lips are under the face it has to slide 5 mm toward its fixed lip so
that lip hooks under the steel, and only then be let go so the latch lip springs out under the other end. Let go too
early and the latch lip comes to rest on top of the face: a false lock that pulls straight out under load."""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import trialkit as tk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BASE = TOP + 0.003
PLY, STEEL, ZINC, RING, WOOD = (0.76, 0.64, 0.45), (0.32, 0.34, 0.36), (0.80, 0.80, 0.76), (0.62, 0.63, 0.66), (0.58, 0.44, 0.28)
Q_X = (math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)  # slide axis along +x

TRACK_X, TRACK_W, TRACK_L = 0.40, 0.127, 0.48
PANEL = (0.20, 0.52, 0.018)
STANDOFF, FACE_T = 0.010, 0.0027
FACE_BOTTOM = PANEL[2] + STANDOFF
FACE_TOP = FACE_BOTTOM + FACE_T
SLOT_L, SLOT_W, PITCH = 0.0622, 0.0147, 0.0508
SLOTS = tuple(round(k * PITCH, 4) for k in range(-4, 5))

PLATE = (0.056, 0.005, 0.008)                        # tongue through the slot, fitting origin at its centre
LIP_H, FIXED_LIP, LATCH_LIP, TRAVEL = 0.003, 0.004, 0.010, 0.010
LIP_DROP = 0.004                                     # lips ride 4 mm clear under the face (the sim's contact margin)
BODY = (0.050, 0.020, 0.014)                         # above the face, bottom at the plate's top
TAB = (0.004, 0.012, 0.012)                          # heel tab and trigger, on top of the body
BODY_TOP = PLATE[2] / 2 + BODY[2]
LATCH_X = PLATE[0] / 2 - TAB[0] / 2                  # trigger stem centre with the latch out
SPRING_K, SPRING_PRE = 800.0, 0.005                  # 4 N with the latch out, 12 N squeezed in
SEATED = FACE_TOP - PLATE[2] / 2                     # fitting origin height with the body on the face
HOOK, SEAT_TOL = 0.002, 0.002
STAND_XY, STAND = (0.22, 0.17), (0.10, 0.05, 0.040)
GROOVE = (0.084, 0.012, 0.010)
START_Z = STAND[2] - PLATE[2] / 2 + 0.001
TARGET = 5                                           # slot the demo uses


def _track_pieces():
    """Two formed legs, a side strip either side of the slots and a bar between each pair of slots, relative to the
    panel's centre."""
    z_leg = PANEL[2] / 2 + STANDOFF / 2
    z_face = PANEL[2] / 2 + STANDOFF + FACE_T / 2
    side = (TRACK_W - SLOT_L) / 2
    out = [("box", (s * (TRACK_W / 2 - 0.005), 0.0, z_leg), (0.010, TRACK_L, STANDOFF), STEEL, True) for s in (-1, 1)]
    out += [("box", (s * (SLOT_L / 2 + side / 2), 0.0, z_face), (side, TRACK_L, FACE_T), STEEL, True) for s in (-1, 1)]
    edges = [-TRACK_L / 2] + [e for y in SLOTS for e in (y - SLOT_W / 2, y + SLOT_W / 2)] + [TRACK_L / 2]
    for y0, y1 in zip(edges[::2], edges[1::2]):
        out.append(("box", (0.0, (y0 + y1) / 2, z_face), (SLOT_L, y1 - y0, FACE_T), STEEL, True))
    return out


def _stand_pieces():
    """Wooden stand with a groove the fitting's tongue hangs in, relative to its 10 mm base slab's centre."""
    h = STAND[2] - GROOVE[2]
    side = (STAND[1] - GROOVE[1]) / 2
    end = (STAND[0] - GROOVE[0]) / 2
    zw = h + GROOVE[2] / 2 - 0.005
    out = [("box", (0.0, 0.0, (0.010 + h) / 2 - 0.005), (STAND[0], STAND[1], h - 0.010), WOOD, True)]
    out += [("box", (0.0, s * (GROOVE[1] / 2 + side / 2), zw), (STAND[0], side, GROOVE[2]), WOOD, True) for s in (-1, 1)]
    out += [("box", (s * (GROOVE[0] / 2 + end / 2), 0.0, zw), (end, GROOVE[1], GROOVE[2]), WOOD, True) for s in (-1, 1)]
    return out


def _ring_pieces():
    """Steel O-ring lying flat off the body's -y side, on its clip (look only)."""
    out = [("box", (0.0, -BODY[1] / 2 - 0.004, PLATE[2] / 2 + BODY[2] / 2), (0.016, 0.008, 0.006), ZINC, False)]
    r, n = 0.022, 12
    cy = -BODY[1] / 2 - 0.008 - r
    for k in range(n):
        a = 2 * math.pi * k / n
        out.append(("box", (r * math.cos(a), cy + r * math.sin(a), PLATE[2] / 2 + 0.004), (0.006, 2 * math.pi * r / n * 1.1,
                                                                                         0.006), RING, False,
                    (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))))
    return out


tk.parts("panel", *_track_pieces())
tk.parts("stand", *_stand_pieces())
tk.parts("fitting",
         ("box", (-PLATE[0] / 2 - FIXED_LIP / 2, 0.0, -PLATE[2] / 2 + LIP_H / 2 - LIP_DROP), (FIXED_LIP, PLATE[1], LIP_H),
          ZINC, True),
         ("box", (-PLATE[0] / 2 - FIXED_LIP / 2, 0.0, -PLATE[2] / 2 + LIP_H - LIP_DROP / 2), (FIXED_LIP, PLATE[1], LIP_DROP),
          ZINC, True),
         ("box", (0.0, 0.0, PLATE[2] / 2 + BODY[2] / 2), BODY, ZINC, True),
         ("box", (-PLATE[0] / 2 + TAB[0] / 2, 0.0, BODY_TOP + TAB[2] / 2), TAB, ZINC, True),
         *_ring_pieces())
tk.parts("latch",
         ("box", (TAB[0] / 2 + LATCH_LIP / 2, 0.0, -PLATE[2] / 2 + LIP_H / 2 - LIP_DROP), (LATCH_LIP, PLATE[1], LIP_H),
          ZINC, True),
         ("box", (0.0, 0.0, BODY_TOP + TAB[2] / 2), TAB, (0.85, 0.20, 0.15), True))
tk.slide("latch", "fitting", (LATCH_X, 0.0, 0.0), TRAVEL, 2.0, rot=Q_X)

OBJECTS = [
    Box("bench", pos=(0.34, 0.0, TOP + 0.0015), size=(0.52, 0.80, 0.003), color=(0.45, 0.47, 0.50)),
    Box("panel", pos=(TRACK_X, 0.0, BASE + PANEL[2] / 2), size=PANEL, color=PLY),
    Box("stand", pos=(*STAND_XY, BASE + 0.005), size=(STAND[0], STAND[1], 0.010), color=WOOD),
    Box("fitting", pos=(*STAND_XY, BASE + START_Z), size=PLATE, mass=0.15, color=ZINC),
    Box("latch", pos=(STAND_XY[0] + LATCH_X, STAND_XY[1], BASE + START_Z), size=(TAB[0], PLATE[1], PLATE[2]),
        mass=0.05, color=ZINC),
]
T = {"n": 0, "sprung": False}


@tk.on_reset
def _new_trial(env, rng):
    T.update(n=0, sprung=False)


def _quat_rot(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def _pose(name):
    base = tk.env().unwrapped
    pose = base.scene[name].data.root_pose_w.torch[0].cpu().numpy()
    return pose[:3] - base.scene.env_origins[0].cpu().numpy() - np.array([0.0, 0.0, BASE]), _quat_rot(*pose[3:7])


def _spring():
    stage, path = tk._PATHS["latch"]
    joint = stage.GetPrimAtPath(f"{path}/joint")
    joint.GetAttribute("drive:linear:physics:stiffness").Set(SPRING_K)
    joint.GetAttribute("drive:linear:physics:targetPosition").Set(SPRING_PRE)


def fitting_state(centre, r, latch):
    """Nearest slot, how far each lip reaches under the face past its slot end, the lips' top relative to the face's
    underside, the body's gap above the face, the latch's travel out, and whether the latch lip sits on the face."""
    slot = min(SLOTS, key=lambda y: abs(centre[1] - y))
    ox = float(centre[0] - TRACK_X)
    out = float((r.T @ (latch - centre))[0] - LATCH_X)
    lip_top = float(centre[2] - PLATE[2] / 2 + LIP_H - LIP_DROP)
    fixed = -SLOT_L / 2 - (ox - PLATE[0] / 2 - FIXED_LIP)
    latch_end = ox + PLATE[0] / 2 + LATCH_LIP + out
    latched = latch_end - SLOT_L / 2
    in_slot = abs(float(centre[1]) - slot) < SLOT_W / 2 and abs(ox) < SLOT_L / 2
    on_face = latched > 0.001 and centre[2] - PLATE[2] / 2 > FACE_TOP - 0.0005
    gap = float(centre[2] - SEATED)
    return slot, fixed, latched, lip_top, gap, out, in_slot, on_face


def _held(pos):
    return any(np.linalg.norm(tcp - pos) < 0.06 and opening < 0.9 for tcp, opening in tk.grippers().values())


def score(objects):
    T["n"] += 1
    if not T["sprung"]:
        _spring()
        T["sprung"] = True
    centre, r = _pose("fitting")
    latch, _ = _pose("latch")
    slot, fixed, latched, lip_top, gap, out, in_slot, on_face = fitting_state(centre, r, latch)
    if T["n"] % 20 == 0 or os.environ.get("ETRACK_DEBUG"):
        print(f"ETRACK t={T['n'] / 20:.1f} slot={slot * 1000:.0f}mm fixed={fixed * 1000:.1f}mm latch={latched * 1000:.1f}"
              f"mm lips={(lip_top - FACE_BOTTOM) * 1000:.1f}mm gap={gap * 1000:.1f}mm out={out * 1000:.1f}mm "
              f"in={in_slot} on_face={on_face}", flush=True)
    if not in_slot or on_face:
        return 0.0
    if lip_top > FACE_BOTTOM:
        return 0.0
    hooked = (fixed >= HOOK) + (latched >= HOOK)
    if hooked < 2:
        return 0.5 if hooked else 0.25
    if gap > SEAT_TOL:
        return 0.5
    return 0.75 if _held(centre + np.array([0.0, 0.0, BASE])) else 1.0


def _open_loop_demo():
    """Left arm: squeeze the heel tab and the trigger together (latch in), lift the fitting off its stand, lower it
    straight down the slot 2 mm toward the latch side of centre, slide 5 mm toward the fixed lip, let go.
    ETRACK_VARIANT picks a shortcut instead: early_release (let go before the slide), release_high (let go over the
    slot before the lips are under the face), off_centre (3 mm along the slot), leave."""
    variant = os.environ.get("ETRACK_VARIANT", "")
    yaw = float(os.environ.get("ETRACK_YAW", math.pi / 2))     # fingers close on the heel tab and the trigger
    place = yaw + float(os.environ.get("ETRACK_TURN", math.pi / 2))   # wrist turn that lines the head up with a slot
    sx, sy = STAND_XY
    pad = BODY_TOP + TAB[2] / 2 + PAD_TIP_PAST_TCP      # TCP above the fitting origin, pads on the tabs' middle
    pick = BASE + START_Z + pad
    drop_x = TRACK_X + float(os.environ.get("ETRACK_DROP", 0.002)) + (0.003 if variant == "off_centre" else 0.0)
    home_x = TRACK_X - 0.0031
    y = SLOTS[TARGET]
    seat = BASE + SEATED + pad
    left = [
        Waypoint(2.0, (sx, sy, pick + 0.05), FINGER_OPEN, "above the fitting", yaw),
        Waypoint(1.5, (sx, sy, pick), FINGER_OPEN, "pads at the heel tab and the trigger", yaw),
        Waypoint(1.0, (sx, sy, pick), FINGER_CLOSED, "squeeze: latch in", yaw),
        Waypoint(1.5, (sx, sy, pick + 0.05), FINGER_CLOSED, "lift out of the stand", yaw),
        Waypoint(3.0, (drop_x, y, seat + 0.03), FINGER_CLOSED, "over the slot, head turned along it", place),
    ]
    if variant == "leave":
        return {"left": left[:2]}
    if variant == "release_high":
        return {"left": left + [Waypoint(1.5, (drop_x, y, seat + 0.002), FINGER_CLOSED, "lips just above the face", place),
                                Waypoint(1.0, (drop_x, y, seat + 0.002), FINGER_OPEN, "let go", place),
                                Waypoint(1.5, (drop_x, y, seat + 0.05), FINGER_OPEN, "up", place)]}
    press = seat - float(os.environ.get("ETRACK_PRESS", 0.004))   # body pressed flat on the face
    left += [Waypoint(2.0, (drop_x, y, press), FINGER_CLOSED, "straight down the slot, pressed flat", place)]
    if variant != "early_release":
        left += [Waypoint(1.5, (home_x, y, press), FINGER_CLOSED, "slide 5 mm: fixed lip under the face", place)]
    end_x = drop_x if variant == "early_release" else home_x
    left += [
        Waypoint(1.0, (end_x, y, press), FINGER_OPEN, "let go: latch springs out", place),
        Waypoint(0.8, (end_x - float(os.environ.get("ETRACK_CLEAR", 0.008)), y, press), FINGER_OPEN,
                 "slide the finger off the trigger so the latch can spring out", place),
        Waypoint(0.6, (end_x, y, press + 0.02), FINGER_OPEN, "up off the tabs", place),
        Waypoint(0.5, (end_x, y, press + 0.02), FINGER_CLOSED, "close the fingers", place),
        Waypoint(1.2, (end_x, y, press - float(os.environ.get("ETRACK_THUMB", 0.008))), FINGER_CLOSED,
                 "press the body flat: latch lip under the face", place),
        Waypoint(1.0, (end_x - float(os.environ.get("ETRACK_THUMB_DX", 0.003)), y,
                       press - float(os.environ.get("ETRACK_THUMB", 0.008))), FINGER_CLOSED,
                 "push it toward the fixed lip", place),
        Waypoint(0.6, (end_x, y, press + 0.02), FINGER_CLOSED, "lift off", place),
        Waypoint(1.5, (end_x, y, seat + 0.06), FINGER_OPEN, "straight up", place),
        Waypoint(2.0, (0.20, 0.25, BASE + 0.15), FINGER_OPEN, "clear", 0.0),
        Waypoint(2.0, (0.20, 0.25, BASE + 0.15), FINGER_OPEN, "hold", 0.0),
    ]
    return {"left": left}



def _servoed():
    """The demo with feedback, as a person lines a fitting up by eye and feel: carry the squeezed fitting over the
    target slot, then up to three times read where the fitting really is and move the hand by the error, then go down
    the slot, slide the fixed lip under the face and let the latch spring out."""
    plan = _open_loop_demo()["left"]
    over = next(i for i, w in enumerate(plan) if w.label.startswith("over the slot"))
    yield {"left": plan[:over + 1]}
    w = plan[over]
    x, y, z, yaw = w.pos[0], w.pos[1], w.pos[2], float(w.yaw)
    target = np.array([TRACK_X + float(os.environ.get("ETRACK_DROP", 0.002)), SLOTS[TARGET]])
    for _ in range(3):
        centre, r = _pose("fitting")
        err = target - centre[:2]
        heading = math.atan2(r[1, 0], r[0, 0])
        turn = math.remainder(heading, math.pi)          # the head lies along x either way round
        print(f"ETRACK servo err=({err[0] * 1000:.1f},{err[1] * 1000:.1f})mm turn={math.degrees(turn):.1f}deg",
              flush=True)
        if np.linalg.norm(err) < 0.0007 and abs(turn) < math.radians(1):
            break
        x, y, yaw = x + float(err[0]), y + float(err[1]), yaw - turn
        yield {"left": [Waypoint(1.2, (x, y, z), FINGER_CLOSED, "line the head up with the slot", yaw)]}
    low = z - 0.03 + 0.014                                # tongue 6 mm above the face (the arm sags ~3 mm here)
    for k in range(4):
        if k:
            centre, r = _pose("fitting")
            err = target - centre[:2]
            turn = math.remainder(math.atan2(r[1, 0], r[0, 0]), math.pi)
            print(f"ETRACK fine err=({err[0] * 1000:.2f},{err[1] * 1000:.2f})mm turn={math.degrees(turn):.1f}deg",
                  flush=True)
            if abs(err[0]) < 0.0003 and abs(err[1]) < 0.001 and abs(turn) < math.radians(1):
                break
            x, y, yaw = x + float(err[0]), y + float(err[1]), yaw - turn
        yield {"left": [Waypoint(1.0, (x, y, low), FINGER_CLOSED, "just above the face, lined up", yaw)]}
    import dataclasses
    variant = os.environ.get("ETRACK_VARIANT", "")
    if variant == "release_high":                         # let go lined up but before the lips are under the face
        yield {"left": [Waypoint(1.0, (x, y, low), FINGER_OPEN, "let go above the face", yaw),
                        Waypoint(1.5, (x, y, low + 0.05), FINGER_OPEN, "up", yaw),
                        Waypoint(2.0, (0.20, 0.25, BASE + 0.15), FINGER_OPEN, "clear", 0.0)]}
        return
    dx, dy = x - w.pos[0], y - w.pos[1]
    at_slot = [v for v in plan[over + 1:] if v.label not in ("clear", "hold")]
    if variant == "no_slide":                             # down the slot and let go without sliding the fixed lip under
        at_slot = [v for v in at_slot if not v.label.startswith(("slide", "push it toward"))]
    away = [v for v in plan[over + 1:] if v.label in ("clear", "hold")]
    yield {"left": [dataclasses.replace(v, pos=(v.pos[0] + dx, v.pos[1] + dy, v.pos[2]), yaw=yaw) for v in at_slot]
           + away}

def demo():
    if os.environ.get("ETRACK_VARIANT") in ("leave", "off_centre", "early_release"):
        return _open_loop_demo()
    return tk.staged(_servoed)()

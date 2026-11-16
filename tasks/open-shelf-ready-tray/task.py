"""Open a shelf-ready tray of cans: tear its front panel off along the perforation.

A brown corrugated shelf-ready tray (B-flute, 3 mm board) stands on the table with its front toward the robot, open at
the top, twelve 15 oz food cans standing in it 4 across and 3 deep. Its front panel comes off along a perforation: across
the front wall above a 4 cm lip, and down a diagonal on each side wall, from the side wall's top edge 9 cm back from the
front down to the lip's top at the front corner (the usual display cut-out). Off, the tray goes straight onto the shelf.

Model
- The removable part is three rigid pieces of board: the front panel and the two triangular side flaps, joined at the
  front corners by the board's creases (hinges about the vertical, sprung straight).
- The perforation is a row of ties: nine along the lip and three down each diagonal, each a ball joint (it carries
  force, not moment, like a 2 mm tie of board) that breaks for good past its strength (F_LIP, F_DIAG). Peeling the panel
  out and down about the lip loads the ties farthest from the lip most, so they break first and the tear runs down the
  diagonals and then along the lip; a straight lift or a sideways pull spreads over all the ties and holds.
- A tie counts as torn once its two ends are more than 2 mm apart.

Scoring (judged at the end; the episode ends once 1.0 has held for SUCCESS_HOLD_S)
  1.0   the panel off (every tie torn) and not in or on the tray, the tray moved no more than 2 cm at any corner, all 12
        cans upright (within 10 degrees) in the tray, both grippers 3 cm or more from the tray, the panel and the flaps,
        and everything still
  0.5   the panel off, but not all of that
  0.25  the tear started (at least one tie torn), the panel still on
  0     nothing torn, or the tray ever tipped more than 10 degrees

Env knobs: DEMO_VARIANT, TRAY_DEBUG=1 (a trace every 5 control steps), F_LIP / F_DIAG (N), HOOK_DEPTH, PEEL_DEG,
PULL_OUT (demo).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, HullShape, allow_matrix_grips, as_numpy, ball, hinge,  # noqa: E402
                     on_reset, on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Open this shelf-ready tray so it can go straight onto the shelf: take its front panel off along the "
               "perforation and leave every can standing in the tray.")

KRAFT = (0.62, 0.47, 0.32)
KRAFT_EDGE = (0.55, 0.41, 0.27)
PRINT = (0.10, 0.08, 0.07)  # printed dashes along the perforation
PANEL_PRINT = (0.13, 0.36, 0.20)  # the printed band on the removable panel
PANEL_LABEL = (0.92, 0.89, 0.78)
LABEL = (0.16, 0.42, 0.22)
LABEL_BAND = (0.93, 0.90, 0.80)
TIN = (0.70, 0.71, 0.73)

# -- the tray (frame: origin at the middle of its front wall's outside face, on the table; x back, y left, z up) -------
B = 0.003  # board
TW, TD, TH = 0.310, 0.241, 0.115  # outside width (y), depth (x), height
LIP = 0.040  # the fixed lip under the front panel
DX = 0.090  # each diagonal's top, this far back from the front
XF = 0.25  # the tray's front face (world x)
TRAY_MASS, PANEL_MASS, FLAP_MASS = 0.26, 0.015, 0.004  # kg (board about 0.55 kg/m2)
F_LIP = float(os.environ.get("F_LIP", "1.2"))  # N: a lip tie's strength
F_DIAG = float(os.environ.get("F_DIAG", "0.6"))  # N: a diagonal tie's strength
CREASE_K = 0.002  # N m per degree: the front corners' creases, sprung straight (the flaps square to the panel)
LIP_TIE_Y = tuple(round(-0.14 + 0.035 * k, 3) for k in range(9))  # on the peel axis: they carry a straight pull
DIAG_TIE_F = (0.2, 0.5, 0.8)  # along each diagonal, from the lip
TORN = 0.002  # a tie's two ends this far apart: torn

# the cans: 15 oz food cans, 4 across x 3 deep, the front row 8 mm behind the front panel
CAN_R, CAN_H, CAN_MASS = 0.0375, 0.111, 0.425
CAN_PITCH = 0.0755
CAN_FRONT_GAP = 0.008
CAN_X = tuple(B + CAN_FRONT_GAP + CAN_R + k * CAN_PITCH for k in range(3))
CAN_Y = (1.5 * CAN_PITCH, 0.5 * CAN_PITCH, -0.5 * CAN_PITCH, -1.5 * CAN_PITCH)

TRAY_POS = np.array([XF, 0.0, TOP])


def _box(lo, hi, color, collide=True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _tri_xz(points_xz, y0, y1, color, collide=True) -> HullShape:
    """A triangle of board in a side wall: (x, z) points, between y0 and y1."""
    return HullShape(tuple((x, y0, z) for x, z in points_xz), tuple((x, y1, z) for x, z in points_xz), color, collide)


def _side_y(s: float) -> tuple[float, float]:
    """The y span of the left (s = 1) or right (s = -1) side wall."""
    return (TW / 2 - B, TW / 2) if s > 0 else (-TW / 2, -TW / 2 + B)


def diag_point(f: float) -> tuple[float, float]:
    """(x, z) in the tray frame at fraction f along a diagonal, from the lip (0) to the top edge (1)."""
    return B + f * (DX - B), LIP + f * (TH - LIP)


def _dash_line(p0, p1, color, n=9) -> list:
    """Printed dashes along a perforation (visual only), on a face: points in the tray frame."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    out = []
    for k in range(n):
        a, b = p0 + (p1 - p0) * (k + 0.2) / n, p0 + (p1 - p0) * (k + 0.8) / n
        out.append(_box(np.minimum(a, b) - 0.0008, np.maximum(a, b) + 0.0008, color, collide=False))
    return out


def _tray_shapes() -> list:
    s = [
        _box((0.0, -TW / 2, 0.0), (TD, TW / 2, B), KRAFT),  # floor
        _box((TD - B, -TW / 2, B), (TD, TW / 2, TH), KRAFT),  # back wall
        _box((0.0, -TW / 2, B), (B, TW / 2, LIP), KRAFT),  # the lip
    ]
    for sgn in (1.0, -1.0):
        y0, y1 = _side_y(sgn)
        s.append(_box((DX, y0, B), (TD - B, y1, TH), KRAFT))  # side wall behind the diagonal
        s.append(_box((B, y0, B), (DX, y1, LIP), KRAFT))  # under the diagonal: below the lip line ...
        s.append(_tri_xz([(B, LIP), (DX, LIP), (DX, TH)], y0, y1, KRAFT))  # ... and the triangle under it
        yo = y1 + 0.0002 if sgn > 0 else y0 - 0.0002
        s += [_box((x - 0.0016, yo - 0.0002, z - 0.0016), (x + 0.0016, yo + 0.0002, z + 0.0016), PRINT, collide=False)
              for x, z in (diag_point((k + 0.5) / 14) for k in range(14))]  # the printed diagonal
    s += _dash_line((-0.0002, -TW / 2 + 0.01, LIP), (-0.0002, TW / 2 - 0.01, LIP), PRINT, n=14)  # the printed lip line
    return s


TRAY = Body.build("tray", pos=tuple(TRAY_POS.tolist()), mass=TRAY_MASS, shapes=_tray_shapes(), friction=(0.5, 0.45),
                  color=KRAFT, contact_offset=0.001, solver_iterations=32)

# -- the removable part ----------------------------------------------------------------------------------------------------
PANEL_POS = TRAY_POS + np.array([B / 2, 0.0, (LIP + TH) / 2])
PANEL = Body.build("panel", pos=tuple(PANEL_POS.tolist()), mass=PANEL_MASS,
                   shapes=[BoxShape((0.0, 0.0, 0.0), (B, TW, TH - LIP), KRAFT),
                           # the panel's print: a coloured band with a light label panel in it (visual only)
                           BoxShape((-B / 2 - 0.0002, 0.0, 0.002), (0.0004, TW - 0.03, 0.040), PANEL_PRINT, collide=False),
                           BoxShape((-B / 2 - 0.0004, 0.0, 0.002), (0.0004, 0.12, 0.026), PANEL_LABEL, collide=False)],
                   friction=(0.5, 0.45), color=KRAFT, contact_offset=0.001, solver_iterations=32)


def flap_pos(s: float) -> np.ndarray:
    y0, y1 = _side_y(s)
    return TRAY_POS + np.array([B, (y0 + y1) / 2, LIP])


def _flap(s: float) -> Body:
    name = "flap_left" if s > 0 else "flap_right"
    pts = [(0.0, 0.0), (0.0, TH - LIP), (DX - B, TH - LIP)]
    return Body.build(name, pos=tuple(flap_pos(s).tolist()), mass=FLAP_MASS,
                      shapes=[_tri_xz(pts, -B / 2, B / 2, KRAFT)], friction=(0.5, 0.45), color=KRAFT,
                      contact_offset=0.001, solver_iterations=32)


FLAPS = {1.0: _flap(1.0), -1.0: _flap(-1.0)}

# ties: (name, segment, anchor in the world)
TIES: list[tuple[str, str, np.ndarray]] = []
for _k, _y in enumerate(LIP_TIE_Y):  # (USD prim names: letters, digits and underscores only)
    TIES.append((f"lip_{_k}", "panel", TRAY_POS + np.array([B / 2, _y, LIP])))
for _s, _f in FLAPS.items():
    for _k, _fr in enumerate(DIAG_TIE_F):
        _x, _z = diag_point(_fr)
        TIES.append((f"diag_{'l' if _s > 0 else 'r'}{_k}", _f.name, TRAY_POS + np.array([_x, flap_pos(_s)[1], _z])))
for _name, _seg, _p in TIES:
    ball(f"tie_{_name}", "tray", _seg, anchor=tuple(_p.tolist()),
         break_force=F_LIP if _name.startswith("lip") else F_DIAG)
for _s, _f in FLAPS.items():
    hinge(f"crease_{'l' if _s > 0 else 'r'}", "panel", _f.name,
          anchor=tuple((TRAY_POS + np.array([B, flap_pos(_s)[1], (LIP + TH) / 2])).tolist()), axis="Z",
          limits_deg=(-120.0, 120.0), spring=CREASE_K, damping=0.0002, target_deg=0.0)

_SEG_POS = {"panel": PANEL_POS, FLAPS[1.0].name: flap_pos(1.0), FLAPS[-1.0].name: flap_pos(-1.0)}
TIE_LOCAL = [(name, seg, p - _SEG_POS[seg], p - TRAY_POS) for name, seg, p in TIES]  # (segment-local, tray-local)


# -- the cans ---------------------------------------------------------------------------------------------------------------
def _can(i: int, x: float, y: float) -> Body:
    return Body.build(f"can_{i}", pos=(XF + x, y, TOP + B + CAN_H / 2 + 0.0005), mass=CAN_MASS,
                      shapes=[CylShape((0.0, 0.0, 0.0), CAN_R, CAN_H, LABEL),
                              CylShape((0.0, 0.0, 0.010), CAN_R + 0.0003, 0.030, LABEL_BAND, collide=False),
                              CylShape((0.0, 0.0, CAN_H / 2 - 0.002), CAN_R - 0.001, 0.004, TIN, collide=False)],
                      friction=(0.4, 0.35), color=LABEL, contact_offset=0.002, solver_iterations=16)


CANS = [_can(i, x, y) for i, (x, y) in enumerate((x, y) for x in CAN_X for y in CAN_Y)]
CAN_START = {c.name: np.array(c.pos) for c in CANS}

OBJECTS = [TRAY, PANEL, FLAPS[1.0], FLAPS[-1.0], *CANS]
SEGMENTS = ("panel", FLAPS[1.0].name, FLAPS[-1.0].name)

# -- state and scoring ------------------------------------------------------------------------------------------------------
TIP_DEG = 10.0  # the tray tipped (latched 0), or a can not upright
MOVED = 0.020  # the tray moved more than this at any corner
LET_GO = 0.030  # every TCP this far from the tray, the panel and the flaps
_TRAY_CORNERS = np.array([[x, y, 0.0] for x in (0.0, TD) for y in (-TW / 2, TW / 2)])
_TRAY_LO, _TRAY_HI = np.array([0.0, -TW / 2, 0.0]), np.array([TD, TW / 2, TH])
_SEG_BOX = {"panel": (-np.array([B, TW, TH - LIP]) / 2, np.array([B, TW, TH - LIP]) / 2),
            FLAPS[1.0].name: (np.array([0.0, -B / 2, 0.0]), np.array([DX - B, B / 2, TH - LIP])),
            FLAPS[-1.0].name: (np.array([0.0, -B / 2, 0.0]), np.array([DX - B, B / 2, TH - LIP]))}


def _fresh() -> dict:
    return {"tcps": None, "steps": 0, "tipped": False, "torn": set(), "first_torn": {}}


_EP = _fresh()
_DEBUG = bool(os.environ.get("TRAY_DEBUG"))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())


def tilt_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


def tie_gap(seg_pos, seg_R, tray_pos, tray_R, p_seg, p_tray) -> float:
    return float(np.linalg.norm((seg_pos + seg_R @ p_seg) - (tray_pos + tray_R @ p_tray)))


def torn_ties(poses: dict) -> set:
    """poses: name -> (pos, R) for the tray and the three segments."""
    tp, tR = poses["tray"]
    out = set()
    for name, seg, p_seg, p_tray in TIE_LOCAL:
        sp, sR = poses[seg]
        if tie_gap(sp, sR, tp, tR, p_seg, p_tray) > TORN:
            out.add(name)
    return out


def _box_distance(local, lo, hi) -> float:
    p = np.asarray(local, float)
    return float(np.linalg.norm(p - np.clip(p, lo, hi)))


def in_or_on_tray(point_world, tray_pos, tray_R) -> bool:
    """A point within the tray's outside, or on top of it (up to 5 cm above its rim), 1 cm of slack all round."""
    p = tray_R.T @ (np.asarray(point_world, float) - tray_pos)
    return bool(np.all(p >= _TRAY_LO - 0.01) and np.all(p <= _TRAY_HI + np.array([0.01, 0.01, 0.05])))


def tray_moved(tray_pos, tray_R) -> float:
    now = tray_pos + _TRAY_CORNERS @ tray_R.T
    start = TRAY_POS + _TRAY_CORNERS
    return float(np.max(np.linalg.norm(now - start, axis=1)))


def can_ok(pos, R, tray_pos, tray_R) -> bool:
    p = tray_R.T @ (np.asarray(pos, float) - tray_pos)
    inside = (B <= p[0] <= TD - B) and abs(p[1]) <= TW / 2 - B and p[2] < B + CAN_H / 2 + 0.01
    return inside and tilt_deg(R) <= TIP_DEG


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, name, origin):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    try:
        _EP["tcps"] = _tcps(base, origin)
    except (KeyError, StopIteration, AttributeError):
        _EP["tcps"] = None
    poses = {n: _pose(base, n, origin) for n in ("tray", *SEGMENTS)}
    if tilt_deg(poses["tray"][1]) > TIP_DEG:
        _EP["tipped"] = True
    torn = torn_ties(poses)
    for name in torn - _EP["torn"]:
        _EP["first_torn"][name] = _EP["steps"]
        print(f"[tray] step {_EP['steps']} tie {name} torn", file=sys.stderr, flush=True)
    _EP["torn"] |= torn
    if _DEBUG and _EP["steps"] % 5 == 0:
        pp, pR = poses["panel"]
        peel = math.degrees(math.atan2(-pR[0, 2], pR[2, 2]))  # the panel's top tipped out (+) about the lip
        tp, tR = poses["tray"]
        tc = [np.round(t, 3).tolist() for t in _EP["tcps"]] if _EP["tcps"] is not None else None
        print(f"[tray] step {_EP['steps']} torn {sorted(_EP['torn'])} panel peel {peel:5.1f} deg pos "
              f"{np.round(pp, 3).tolist()} tray moved {tray_moved(tp, tR) * 1000:.1f}mm tilt {tilt_deg(tR):.1f} "
              f"tcps {tc}", file=sys.stderr, flush=True)


def status(objects) -> dict:
    tray = objects["tray"]
    tp, tR = np.asarray(tray.pos, float), rotation(tray.quat_xyzw)
    poses = {"tray": (tp, tR)}
    for n in SEGMENTS:
        poses[n] = (np.asarray(objects[n].pos, float), rotation(objects[n].quat_xyzw))
    torn = torn_ties(poses) | _EP["torn"]
    removed = not any(in_or_on_tray(poses[n][0] + poses[n][1] @ ((_SEG_BOX[n][0] + _SEG_BOX[n][1]) / 2), tp, tR)
                      for n in SEGMENTS)
    cans = all(can_ok(objects[c.name].pos, rotation(objects[c.name].quat_xyzw), tp, tR) for c in CANS)
    tcps = _EP["tcps"]
    let_go = tcps is not None
    if tcps is not None:
        for t in tcps:
            if _box_distance(tR.T @ (t - tp), _TRAY_LO, _TRAY_HI) < LET_GO:
                let_go = False
            for n in SEGMENTS:
                sp, sR = poses[n]
                if _box_distance(sR.T @ (t - sp), *_SEG_BOX[n]) < LET_GO:
                    let_go = False
    still = tray.resting() and all(objects[n].resting() for n in SEGMENTS) and all(objects[c.name].resting()
                                                                                   for c in CANS)
    return {"torn": torn, "off": len(torn) == len(TIES), "removed": removed, "tray_ok": tray_moved(tp, tR) <= MOVED,
            "cans_ok": cans, "let_go": let_go, "still": still, "tipped": _EP["tipped"] or tilt_deg(tR) > TIP_DEG}


def points(st: dict) -> float:
    if st["tipped"]:
        return 0.0
    if st["off"]:
        full = st["removed"] and st["tray_ok"] and st["cans_ok"] and st["let_go"] and st["still"]
        return 1.0 if full else 0.5
    return 0.25 if st["torn"] else 0.0


def score(objects) -> float:
    """1.0: the panel off and away from the tray, tray in place, all cans upright in it, grippers clear, all still.
    0.5: the panel off otherwise. 0.25: the tear started. 0: nothing torn, or the tray tipped."""
    return points(status(objects))


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------------
# The left hand turns its fingers to close along x, opens them 6 mm either side of the TCP and lowers them over the
# middle of the front panel's top edge: the back finger goes down behind the panel into the gap between the two middle
# front cans, the front finger in front of it. It pinches the top 2 cm of the panel and peels it out and down about the
# lip, turning the wrist with it: the diagonals tear from the top. Holding the panel at about 55 degrees, it pulls it on
# out (aimed well past: the arm gives about 200 N/m sideways), which tears the lip, carries the panel off to the left,
# lowers it to the table and lets go. The right hand stays parked.
allow_matrix_grips()

TIP = 0.013  # the pad tips this far below the TCP
APPROACH_V = -0.006  # finger setting going down: each pad 6 mm from the TCP along x (the back finger's body has to
#                      fit between the cans: the gap is 3 cm wide or more only within 19 mm of the front wall)
PINCH = FINGER_CLOSED  # then closed on the 3 mm board: about 7 N of squeeze
PINCH_DEPTH = float(os.environ.get("PINCH_DEPTH", "0.020"))  # the pad tips this far below the panel's top edge
PEEL_DEG = float(os.environ.get("PEEL_DEG", "55"))  # the panel peeled this far out and down about the lip ...
PULL_OUT = float(os.environ.get("PULL_OUT", "0.04"))  # ... then pulled straight out by this (commanded; tears the lip)
R_HOOK = grip_rotation(-math.pi / 2)  # straight down, the fingers closing along x (-90: within the left wrist's range)
PARK = {"left": (0.15, 0.30, TOP + 0.20), "right": (0.15, -0.30, TOP + 0.20)}
DROP = (0.15, 0.24)  # where the torn-off panel is put down, on the table left of the tray
_LIP_AXIS = np.array([XF + B / 2, 0.0, TOP + LIP])


def _ry(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def pinch_tcp() -> np.ndarray:
    """The TCP centred on the board, the pad tips PINCH_DEPTH below the panel's top edge."""
    return np.array([XF + B / 2, 0.0, TOP + TH - PINCH_DEPTH + TIP])


def peel_pose(deg: float) -> tuple[np.ndarray, np.ndarray]:
    """(TCP, rotation) with the pinched panel swung ``deg`` out and down about the lip, the wrist turned with it."""
    R = _ry(-deg)  # about +y, negative: the top goes out toward the robot
    return _LIP_AXIS + R @ (pinch_tcp() - _LIP_AXIS), R @ R_HOOK


def hook_tcp() -> np.ndarray:  # kept for the offline gap check: where the back pad is while going down
    return pinch_tcp() + np.array([APPROACH_V, 0.0, 0.0])


BACK_CLEAR = -APPROACH_V - B / 2  # the back pad this far behind the panel's inside face going down


def _w(t, pos, finger, label, R=R_HOOK) -> Waypoint:
    return Waypoint(float(t), tuple(float(v) for v in pos), finger, label, R)


def plan_left(variant: str) -> list[Waypoint]:
    pinch = pinch_tcp()
    wps = [
        _w(1.5, (0.24, 0.20, TOP + 0.22), APPROACH_V, "toward the tray"),
        _w(1.5, pinch + [0.0, 0.0, 0.06], APPROACH_V, "over the middle of the front panel's top edge"),
        _w(1.5, pinch, APPROACH_V, "fingers down either side of the panel"),
        _w(0.6, pinch, PINCH, "pinch the panel's top edge"),
        _w(0.4, pinch, PINCH, "steady"),
    ]
    if variant == "lift":  # a straight pull up, in the panel's plane: it holds
        return wps + [_w(1.5, pinch + [0.0, 0.0, 0.008], PINCH, "pull straight up"),
                      _w(1.5, pinch + [0.0, 0.0, 0.008], PINCH, "hold"),
                      _w(0.6, pinch, APPROACH_V, "let go"),
                      _w(1.2, pinch + [0.0, 0.0, 0.07], APPROACH_V, "up out of the gap"),
                      _w(2.0, PARK["left"], FINGER_OPEN, "park", grip_rotation(0.0))]
    stop = 3.0 if variant == "start_only" else PEEL_DEG
    for d in [d for d in (3, 5, 10, 15, 20, 30, 40, 50, 55) if d <= stop]:
        p, R = peel_pose(d)
        wps.append(_w(0.8, p, PINCH, "peel the panel out and down about the lip", R))
    p, R = peel_pose(stop)
    if variant == "start_only":
        return wps + [_w(60.0, p, PINCH, "hold the panel part-peeled", R)]
    out = p + np.array([-PULL_OUT, 0.0, 0.0])
    wps += [_w(1.5, out, PINCH, "pull it straight out: the lip tears", R), _w(0.8, out, PINCH, "steady", R)]
    if variant == "keep_holding":
        return wps + [_w(60.0, out, PINCH, "keep holding the panel", R)]
    if variant == "drop_in_tray":  # torn off, but put down on top of the cans
        over = np.array([XF + 0.08, 0.0, TOP + TH + 0.10])
        down = np.array([XF + 0.08, 0.0, TOP + TH + 0.05])
    else:
        over = np.array([DROP[0], DROP[1], TOP + 0.10])
        down = np.array([DROP[0], DROP[1], TOP + 0.045])
    return wps + [
        _w(1.2, out + [0.0, 0.0, 0.06], PINCH, "lift the panel clear", R),
        _w(2.0, over, PINCH, "carry it off to the left", R),
        _w(1.0, down, PINCH, "lower it to the table", R),
        _w(0.6, down, FINGER_OPEN, "let go", R),
        _w(1.2, down + [0.0, 0.0, 0.08], FINGER_OPEN, "lift away", R),
        _w(2.0, PARK["left"], FINGER_OPEN, "park", grip_rotation(0.0)),
    ]


def _length(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    """DEMO_VARIANT: full (1.0); nothing (0); lift (pinched and pulled straight up, in the panel's plane: holds, 0);
    start_only (peeled 3 degrees and held: the tear started, 0.25 if it hasn't run all the way); keep_holding (torn off,
    still held: 0.5); drop_in_tray (torn off and put down on the cans: 0.5)."""
    variant = os.environ.get("DEMO_VARIANT", "full")
    if variant == "nothing":
        left = [_w(3.0, PARK["left"], FINGER_OPEN, "park", grip_rotation(0.0))]
    else:
        left = plan_left(variant)
    total = _length(left)
    right = [_w(2.0, PARK["right"], FINGER_OPEN, "park", grip_rotation(0.0)),
             _w(max(total - 2.0, 0.5), PARK["right"], FINGER_OPEN, "wait", grip_rotation(0.0))]
    for arm in (left, right):
        w = arm[-1]
        arm.append(_w(2.5, w.pos, w.finger, "wait: the trial ends once the full score has held 1 s", w.yaw))
    return {"left": left, "right": right}

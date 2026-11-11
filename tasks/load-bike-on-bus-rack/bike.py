"""A 1:6 bicycle as one rigid body made of boxes (a partkit Part). Bike frame: origin on the ground midway between
the axles, +y toward the front wheel, x across the bike (the axles' direction), z up. Wheels are 16-segment tyre
rings with spokes; the wheels don't spin (the bike is lifted, never rolled). No fenders; a front rim-brake caliper
sits on the fork crown. Pure geometry: no simulator needed."""

import math

import numpy as np

R = 0.057  # wheel radius to the tyre's outer face (11.4 cm across: a 68 cm wheel at 1:6)
TYRE_W, TYRE_T = 0.010, 0.008  # tyre width (x) and radial thickness
AXLE_Y = 0.0875  # front axle at +AXLE_Y, rear at -AXLE_Y (17.5 cm wheelbase)
SEGMENTS = 16
MASS = 0.30
FRICTION = 1.0

FRAME = (0.20, 0.42, 0.62)  # blue frame
TYRE = (0.07, 0.07, 0.08)
METAL = (0.70, 0.71, 0.73)
BLACK = (0.10, 0.10, 0.11)
SADDLE = (0.16, 0.12, 0.10)

TOP_TUBE = dict(y0=-0.038, y1=0.079, z=0.130, w=0.016, h=0.020)  # the grip: a flat bar, centre height z
CROWN = dict(y=AXLE_Y, half_y=0.010, z0=0.118, z1=0.128, half_x=0.015)  # fork crown and brake caliper
BB = (-0.016, 0.047)  # bottom bracket (y, z)
HEAD = (0.085, 0.128, 0.152)  # head tube (y, z bottom, z top)
SEAT_TOP = (-0.030, 0.140)
BARS = dict(y=0.072, z=0.162, half_x=0.045)
LEG_X = TYRE_W / 2 + 0.0025 + 0.002  # fork legs and stays: centre x, outside the tyre


def _qx(angle):
    """(w, x, y, z) quaternion of a rotation about the bike's x axis."""
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)


def _strut(a, b, x, w, color):
    """A box from (y, z) point a to b in the bike's plane, at x, w thick (x and the cross direction)."""
    (ya, za), (yb, zb) = a, b
    length = math.hypot(yb - ya, zb - za)
    return ((x, (ya + yb) / 2, (za + zb) / 2), (w, length, w), color, _qx(math.atan2(zb - za, yb - ya)))


def _wheel(yc):
    out = []
    seg = 2 * R * math.tan(math.pi / SEGMENTS) + 0.001
    for k in range(SEGMENTS):
        th = -math.pi / 2 + k * 2 * math.pi / SEGMENTS  # the first segment lies flat at the bottom
        rc = R - TYRE_T / 2
        out.append(((0.0, yc + rc * math.cos(th), R + rc * math.sin(th)), (TYRE_W, seg, TYRE_T), TYRE,
                    _qx(th + math.pi / 2)))
    for k in range(4):  # spokes
        out.append(((0.0, yc, R), (0.002, 2 * (R - TYRE_T), 0.003), METAL, _qx(k * math.pi / 4)))
    out.append(((0.0, yc, R), (0.018, 0.012, 0.012), METAL, (1.0, 0.0, 0.0, 0.0)))  # hub
    return out


def boxes():
    """Every box of the bike: (centre, size, colour, rotation wxyz), bike frame."""
    ident = (1.0, 0.0, 0.0, 0.0)
    tt = TOP_TUBE
    out = _wheel(-AXLE_Y) + _wheel(AXLE_Y)
    out += [
        ((0.0, (tt["y0"] + tt["y1"]) / 2, tt["z"]), (tt["w"], tt["y1"] - tt["y0"], tt["h"]), FRAME, ident),  # top tube
        ((0.0, HEAD[0], (HEAD[1] + HEAD[2]) / 2), (0.012, 0.014, HEAD[2] - HEAD[1]), FRAME, ident),  # head tube
        _strut((HEAD[0], HEAD[1] + 0.004), BB, 0.0, 0.010, FRAME),  # down tube
        _strut(BB, SEAT_TOP, 0.0, 0.010, FRAME),  # seat tube
        ((0.0, BB[0], BB[1]), (0.020, 0.014, 0.014), FRAME, ident),  # bottom bracket
        ((0.0, CROWN["y"], (CROWN["z0"] + CROWN["z1"]) / 2),
         (2 * CROWN["half_x"], 2 * CROWN["half_y"], CROWN["z1"] - CROWN["z0"]), BLACK, ident),  # crown + brake caliper
        ((0.0, SEAT_TOP[0] - 0.002, 0.148), (0.006, 0.006, 0.016), METAL, ident),  # seat post
        ((0.0, SEAT_TOP[0] - 0.006, 0.158), (0.022, 0.040, 0.008), SADDLE, ident),  # saddle
        ((0.0, 0.080, 0.157), (0.008, 0.016, 0.010), METAL, ident),  # stem
        ((0.0, BARS["y"], BARS["z"]), (2 * BARS["half_x"], 0.008, 0.008), METAL, ident),  # handlebar
        ((BARS["half_x"] - 0.010, BARS["y"], BARS["z"]), (0.020, 0.010, 0.010), BLACK, ident),  # grips
        ((-BARS["half_x"] + 0.010, BARS["y"], BARS["z"]), (0.020, 0.010, 0.010), BLACK, ident),
        ((0.016, BB[0] + 0.006, BB[1] - 0.012), (0.004, 0.006, 0.030), BLACK, _qx(0.4)),  # crank
    ]
    for s in (-1, 1):
        x = s * LEG_X
        out += [
            ((x, AXLE_Y, (R + CROWN["z0"]) / 2), (0.004, 0.007, CROWN["z0"] - R + 0.002), FRAME, ident),  # fork leg
            _strut(BB, (-AXLE_Y, R), x, 0.005, FRAME),  # chain stay
            _strut((SEAT_TOP[0], SEAT_TOP[1] - 0.012), (-AXLE_Y, R), x, 0.005, FRAME),  # seat stay
        ]
    return out


def _rot(wxyz):
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def centre_of_mass():
    """Volume-weighted centre of the boxes (PhysX gives the body uniform density over its colliders)."""
    v = np.array([s[0] * s[1] * s[2] for _, s, _, _ in boxes()])
    c = np.array([c for c, _, _, _ in boxes()])
    return tuple((v @ c / v.sum()).tolist())


def extent():
    """The bike's bounding box in its own frame, (lo, hi)."""
    pts = []
    for c, s, _, q in boxes():
        R_ = _rot(q)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(np.array(c) + R_ @ (np.array(s) / 2 * (sx, sy, sz)))
    p = np.array(pts)
    return tuple(p.min(0).tolist()), tuple(p.max(0).tolist())


OUTLINE = extent()
WHEELS = {"front": (0.0, AXLE_Y, R), "rear": (0.0, -AXLE_Y, R)}
COM = centre_of_mass()
GRIP_Y = round(COM[1], 3)  # where a gripper takes the top tube: over the centre of mass

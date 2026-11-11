"""The wheel and the fork as parts.py parts. Pure Python and NumPy (parts.py imports only yam_arena.task_file).

WHEEL: one articulation. Its body ("base") is the axle with its locknuts and the adjusting nut. Three hinges:
- "spin": rim, tyre, spokes and hub shell turn freely about the axle (a real wheel spins on its bearings while the
  axle stays put);
- "aim": the skewer with the lever's cam housing turns about the axle, held only by friction (see mechanism.py);
- "cam": the lever, on the cam housing, swings from open (pointing straight out along the axle, toward the robot)
  to closed (lying flat along the fork). Its angle is -closure.
FORK: fixed. A fork held upside down in a workshop stand: base plate, crown, two blades, and at the top two dropouts,
each a slot open upward with a flared mouth. A white reflector on the crown's front face marks the bike's front (+y).
"""

import math

import layout as L
import parts

BLACK, SILVER, STEEL, DARK = (0.06, 0.06, 0.07), (0.78, 0.79, 0.80), (0.55, 0.56, 0.58), (0.20, 0.20, 0.22)
LEVER_BLACK = (0.10, 0.10, 0.11)
PAINT = (0.62, 0.10, 0.09)  # the fork's paint
STAND = (0.35, 0.36, 0.38)
WHITE = (0.95, 0.95, 0.93)
RUBBER = (0.18, 0.18, 0.19)


def _about_x(angle):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def _ring(r0, r1, width, color):
    """Flat segments of a ring in the y-z plane between radii r0 and r1; segment 6 is centred on top."""
    out = []
    n = L.SEGMENTS
    for k in range(n):
        th = 2 * math.pi * k / n
        rm = (r0 + r1) / 2
        out.append(parts.box((width, r1 - r0, 2 * r1 * math.tan(math.pi / n)), at=(0.0, rm * math.cos(th), rm * math.sin(th)),
                             color=color, rot=_about_x(th)))
    return out


def _spokes(n=8):
    out = []
    length = L.R_IN - L.HUB_R
    rm = (L.R_IN + L.HUB_R) / 2
    for k in range(n):
        th = 2 * math.pi * (k + 0.5) / n
        side = 0.005 if k % 2 else -0.005  # laced to both hub flanges
        out.append(parts.box((0.0015, length, 0.0015), at=(side, rm * math.cos(th), rm * math.sin(th)), color=SILVER,
                             rot=_about_x(th)))
    return out


# -- the wheel ---------------------------------------------------------------------------------------------------------
_ax0, _ax1 = L.CAP_X
AXLE_SHAPES = [
    parts.box((2 * L.AXLE_HALF, L.AXLE_SIDE, L.AXLE_SIDE), color=STEEL),
    *[parts.box((0.003, 0.016, 0.016), at=(s * (L.LOCKNUT_X - 0.0015), 0.0, 0.0), color=DARK) for s in (1, -1)],
    parts.cylinder(L.CAP_R, _ax1 - _ax0, at=((_ax0 + _ax1) / 2, 0.0, 0.0), color=SILVER, axis="x"),  # adjusting nut
]
SPIN = parts.Hinge(
    "spin", pivot=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0), lower=-3.0, upper=3.0, mass=L.MASS["spin"], spring=0.0,
    damping=0.0005,
    shapes=[*_ring(L.R_RIM, L.R_OUT, L.TYRE_W, BLACK), *_ring(L.R_IN, L.R_RIM, L.RIM_W, SILVER), *_spokes(),
            parts.cylinder(L.HUB_R, 2 * (L.LOCKNUT_X - 0.003), color=SILVER, axis="x")],
)
AIM = parts.Hinge(
    "aim", pivot=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0), lower=-math.radians(L.AIM_LIMIT),
    upper=math.radians(L.AIM_LIMIT), mass=L.MASS["aim"], spring=L.AIM["spring"], damping=L.AIM["damping"],
    shapes=[parts.cylinder(L.CAP_R, _ax1 - _ax0, at=(-(_ax0 + _ax1) / 2, 0.0, 0.0), color=SILVER, axis="x")],
)
_arm0, _arm1, _arm_w = L.LEVER["arm"]
_pad0, _pad1, _pad_w = L.LEVER["paddle"]
CAM = parts.Hinge(
    "cam", parent="aim", pivot=(L.PIVOT_X, 0.0, 0.0), axis=(0.0, 1.0, 0.0), lower=-math.pi / 2, upper=0.05,
    mass=L.MASS["cam"], spring=L.CAM["spring"], damping=L.CAM["damping"],
    shapes=[parts.box((_arm1 - _arm0, _arm_w, L.LEVER["thick"]), at=(L.PIVOT_X - (_arm0 + _arm1) / 2, 0.0, 0.0),
                      color=LEVER_BLACK),
            parts.box((_pad1 - _pad0, _pad_w, L.LEVER["thick"]), at=(L.PIVOT_X - (_pad0 + _pad1) / 2, 0.0, 0.0),
                      color=LEVER_BLACK)],
)
WHEEL = parts.Part("wheel", pos=L.WHEEL_START, mass=L.MASS["base"], shapes=AXLE_SHAPES, joints=[SPIN, AIM, CAM],
                   self_collide=False)


# -- the fork ----------------------------------------------------------------------------------------------------------
def _dropout(s):
    """The dropout plate at x = s * PLATE_X: plate, two jaws either side of the slot, and two flared walls."""
    x = s * L.PLATE_X
    out = [parts.box((L.PLATE_T, 2 * L.JAW_OUT, L.PLATE_DEPTH), at=(x, 0.0, L.SLOT_END - L.PLATE_DEPTH / 2), color=PAINT)]
    lean = math.radians(L.FLARE_DEG)
    wall = L.FLARE_H / math.cos(lean)
    for t in (1, -1):
        out.append(parts.box((L.PLATE_T, L.JAW_OUT - L.SLOT_HALF, L.SLOT_STRAIGHT),
                             at=(x, t * (L.SLOT_HALF + L.JAW_OUT) / 2, L.SLOT_END + L.SLOT_STRAIGHT / 2), color=PAINT))
        # a wall whose inner face runs from the slot's edge up and out to the mouth's edge
        ya, za = t * L.SLOT_HALF, L.SLOT_END + L.SLOT_STRAIGHT
        yb, zb = t * L.MOUTH_HALF, L.MOUTH_TOP
        ny, nz = t * math.cos(lean), -math.sin(lean)  # outward normal of the inner face
        th = 0.005
        out.append(parts.box((L.PLATE_T, th, wall), at=(x, (ya + yb) / 2 + ny * th / 2, (za + zb) / 2 + nz * th / 2),
                             color=PAINT, rot=_about_x(-t * lean)))
    return out


_blade_lo, _blade_hi = L.CROWN["z1"], L.SLOT_END - L.PLATE_DEPTH
B = L.BASE
FORK_SHAPES = [
    parts.box((B["x1"] - B["x0"], 2 * B["half_y"], B["h"]), at=((B["x0"] + B["x1"]) / 2, 0.0, B["h"] / 2), color=STAND),
    parts.box((2 * L.CROWN["half_x"], 2 * L.CROWN["half_y"], L.CROWN["z1"] - L.CROWN["z0"]),
              at=(0.0, 0.0, (L.CROWN["z0"] + L.CROWN["z1"]) / 2), color=PAINT),
    *[parts.box((L.PLATE_T, 0.014, _blade_hi - _blade_lo), at=(s * L.PLATE_X, 0.0, (_blade_lo + _blade_hi) / 2),
                color=PAINT) for s in (1, -1)],
    *_dropout(1), *_dropout(-1),
    parts.box((0.006, 0.025, 0.004), at=(0.0, L.CROWN["half_y"] + 0.0125, 0.010), color=DARK),  # reflector bracket
    parts.box((0.016, 0.004, 0.016), at=(0.0, 0.037, 0.014), color=WHITE),  # front reflector
]
FORK = parts.Part("fork", pos=(L.FORK_XY[0], L.FORK_XY[1], L.TOP), mass=None, shapes=FORK_SHAPES,
                  friction=(L.FORK_FRICTION, L.FORK_FRICTION), combine="min")

# -- the wheel stand: two fixed rubber blocks either side of the tyre's bottom -------------------------------------------
STAND_X = [L.STAND_XY[0] + s * (L.TYRE_W / 2 + L.STAND_GAP + L.STAND_BLOCK[0] / 2) for s in (1, -1)]

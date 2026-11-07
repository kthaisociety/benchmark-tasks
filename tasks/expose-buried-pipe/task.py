"""A hand-digging trainer on the station table, in front of the right arm. A fixed open-topped soil box with
grey walls runs forward from the arms: 180 mm long and 54 mm wide inside, its side walls and far wall 60 mm
high, its near wall (nearest the arms) only 24 mm high. A yellow gas pipe, 25 mm across, lies on the box
floor along its length. The pipe is buried under 156 brown 12 mm soil cubes, about 25 mm deep over its top;
the last 15 mm before the far wall is empty. White locate marks across both side walls show the stretch
where the pipe must be uncovered. A grey spoil board lies on the table just past the near wall, toward the
arms. A flat square spade stands in a grey holder to the right of the box: a green 16 x 24 mm handle over a
3 mm steel blade, 42 mm wide and 35 mm tall, its faces toward and away from the arms. Soil dragged toward
the arms goes over the low near wall onto the spoil board. Dig the soil off the top of the pipe between the
white marks until the top of the pipe is bare along at least 8 cm. The spade must never touch the pipe.
"""

from dataclasses import dataclass
import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "Dig the soil off the top of the yellow pipe between the white marks until the top of the pipe is bare "
    "along at least 8 cm. Never let the spade touch the pipe."
)

GREY = (0.55, 0.57, 0.58)
DARK = (0.33, 0.35, 0.37)
YELLOW = (0.98, 0.78, 0.10)
WHITE = (0.94, 0.94, 0.92)
SOIL = (0.42, 0.29, 0.18)
STEEL = (0.62, 0.65, 0.68)
GREEN = (0.20, 0.42, 0.30)
BOARD = (0.40, 0.42, 0.44)

# Soil box: inner channel, world frame (m).
BOX_X = (0.210, 0.390)
BOX_Y = (-0.237, -0.183)
BOX_H = 0.060
NEAR_WALL_H = 0.024  # just under the pipe's top, so soil above the pipe can be dragged out over it
WALL = 0.006
CHANNEL_Y = 0.5 * (BOX_Y[0] + BOX_Y[1])

# Pipe: a cylinder along world x on the box floor (the table), from the near wall out through the far wall.
PIPE_R = 0.0125
PIPE_X = (BOX_X[0], 0.420)
PIPE_AXIS = (CHANNEL_Y, TOP + PIPE_R)  # (y, z) of the axis
CROWN_Z = TOP + 2 * PIPE_R

# The marked stretch whose top must be uncovered.
MARKS_X = (0.250, 0.350)

# Soil: 12 mm wooden cubes, 13 rows along the channel, 4 columns across. The outer columns stand four high
# beside the pipe; the inner two columns stand two high on top of it.
CUBE = 0.012
SOIL_R = CUBE / 2  # half-width, for the "lies over the pipe" test
CUBE_MASS = 0.0012
ROWS_X = [round(BOX_X[0] + 0.0065 + 0.0127 * i, 5) for i in range(13)]
OUTER_Y = (-0.0195, 0.0195)
INNER_Y = (-0.0065, 0.0065)
OUTER_Z = [0.0061 + 0.0122 * k for k in range(4)]
INNER_Z = [0.0316, 0.0438]
FILL = [
    (x, round(CHANNEL_Y + dy, 5), round(TOP + z, 5))
    for x in ROWS_X
    for dys, zs in ((OUTER_Y, OUTER_Z), (INNER_Y, INNER_Z))
    for dy in dys
    for z in zs
]

# Spade, in its own frame: the origin is the grasp point (the TCP when held), z up along the handle.
SPADE_MASS = 0.070
HOLDER_XY = (0.300, -0.330)
SPADE_HOME = (HOLDER_XY[0], HOLDER_XY[1], TOP + 0.107)  # blade edge resting on the holder's slot floor
EDGE_Z = -0.105  # the blade's bottom edge


@dataclass(frozen=True)
class Part:
    name: str
    center: tuple
    size: tuple
    color: tuple
    collides: bool = True
    shape: str = "box"  # "box" or "cylinder_x" (size = (length, radius, radius))


SPADE_PARTS = [
    Part("handle", (0.0, 0.0, -0.025), (0.016, 0.024, 0.090), GREEN),
    Part("blade", (0.0, 0.0, -0.0875), (0.003, 0.042, 0.035), STEEL),
]

OBJECTS = []
PARTS = {}


def body(name, pos, parts, mass=None):
    # Box is the Arena registry entry; run.py authors the compound colliders from PARTS.
    OBJECTS.append(Box(name=name, pos=pos, size=parts[0].size, color=parts[0].color, mass=mass))
    PARTS[name] = parts


_cx = 0.5 * (BOX_X[0] + BOX_X[1])
_len = BOX_X[1] - BOX_X[0] + 2 * WALL
_wid = BOX_Y[1] - BOX_Y[0] + 2 * WALL
_marks = [
    Part(f"mark_{side}_{i}", (x - _cx, dy, BOX_H + 0.0006), (0.004, WALL + 0.002, 0.0012), WHITE, False)
    for i, x in enumerate(MARKS_X)
    for side, dy in (("l", 0.5 * _wid - 0.5 * WALL), ("r", -0.5 * _wid + 0.5 * WALL))
]
body("soil_box", (_cx, CHANNEL_Y, TOP), [
    Part("wall_l", (0.0, 0.5 * _wid - 0.5 * WALL, 0.5 * BOX_H), (_len, WALL, BOX_H), GREY),
    Part("wall_r", (0.0, -0.5 * _wid + 0.5 * WALL, 0.5 * BOX_H), (_len, WALL, BOX_H), GREY),
    Part("wall_near", (-0.5 * _len + 0.5 * WALL, 0.0, 0.5 * NEAR_WALL_H), (WALL, _wid, NEAR_WALL_H), GREY),
    Part("wall_far", (0.5 * _len - 0.5 * WALL, 0.0, 0.5 * BOX_H), (WALL, _wid, BOX_H), GREY),
    *_marks,
])
body("gas_pipe", (0.5 * (PIPE_X[0] + PIPE_X[1]), PIPE_AXIS[0], PIPE_AXIS[1]), [
    Part("pipe", (0.0, 0.0, 0.0), (PIPE_X[1] - PIPE_X[0], PIPE_R, PIPE_R), YELLOW, shape="cylinder_x"),
])
body("spoil_board", (0.150, CHANNEL_Y, TOP + 0.002), [
    Part("board", (0.0, 0.0, 0.0), (0.100, 0.120, 0.004), BOARD),
])
body("spade_holder", (HOLDER_XY[0], HOLDER_XY[1], TOP), [
    Part("jaw_near", (-0.012, 0.0, 0.010), (0.019, 0.060, 0.020), DARK),
    Part("jaw_far", (0.012, 0.0, 0.010), (0.019, 0.060, 0.020), DARK),
    Part("slot_floor", (0.0, 0.0, 0.001), (0.005, 0.060, 0.002), DARK),
])
body("spade", SPADE_HOME, SPADE_PARTS, SPADE_MASS)
# The soil cubes are rigid bodies authored by run.py under this fixed placeholder (no collider of its own).
body("soil", (_cx, CHANNEL_Y, TOP + 0.03), [
    Part("placeholder", (0.0, 0.0, 0.0), (0.001, 0.001, 0.001), SOIL, False),
])


# -- scripted demo ----------------------------------------------------------------------------------------


@dataclass
class TiltWaypoint(Waypoint):
    pitch: float = 0.0  # rad about world y, applied after the yaw (run.py); the demo keeps the blade upright


YAW = math.pi / 2  # fingers close along world x, across the handle's 16 mm faces


GRIP_ABOVE = 0.003  # the demo closes the fingers this far above the spade's grasp point


def tcp_for_edge(edge):
    """The TCP that puts the middle of the upright blade's bottom edge at ``edge`` (spade held as the demo
    holds it)."""
    return (edge[0], edge[1], edge[2] - EDGE_Z + GRIP_ABOVE)


# Height of the blade's edge above the pipe's top on each pass (m), shallow last. The arm settles about
# 1.5 mm below its target with the spade in hand, and the blade rides up as it drags.
PASSES = (0.012, 0.006, 0.005, 0.005, 0.005, 0.005, 0.005)
TRAVEL = TOP + 0.200
POCKET_X = BOX_X[1] - 0.008  # the empty strip before the far wall, where the blade goes in


def demo():
    """Right arm: take the spade, drag the cover toward the arms and over the low near wall in passes, each
    shallower than the last, finishing 3 mm over the pipe; put the spade back in its holder and withdraw."""
    hx, hy, hz = SPADE_HOME

    def wp(duration, pos, label, finger=FINGER_CLOSED):
        return TiltWaypoint(duration, tuple(pos), finger, label, YAW, 0.0)

    right = [
        wp(2.0, (hx, hy, TRAVEL), "above spade", finger=FINGER_OPEN),
        wp(1.5, (hx, hy, hz + GRIP_ABOVE), "around handle", finger=FINGER_OPEN),
        wp(0.8, (hx, hy, hz + GRIP_ABOVE), "grip"),
        wp(1.5, (hx, hy, TRAVEL), "lift spade"),
    ]
    end_x = BOX_X[0] - 0.075
    for i, depth in enumerate(PASSES):
        z = CROWN_Z + depth
        tag = f"pass {i + 1}"
        right += [
            wp(1.5, tcp_for_edge((POCKET_X, CHANNEL_Y, TOP + BOX_H + 0.010)), f"{tag}: over the far end"),
            wp(1.0, tcp_for_edge((POCKET_X, CHANNEL_Y, z + 0.015)), f"{tag}: blade down"),
            wp(1.0, tcp_for_edge((POCKET_X, CHANNEL_Y, z)), f"{tag}: ease to depth"),
            wp(4.5, tcp_for_edge((end_x, CHANNEL_Y, z)), f"{tag}: drag toward the arms"),
            wp(1.0, tcp_for_edge((end_x, CHANNEL_Y, TOP + BOX_H + 0.010)), f"{tag}: lift"),
        ]
    right += [
        wp(1.5, (0.300, -0.330, TRAVEL), "hold the spade clear"),
        wp(3.0, (0.300, -0.330, TRAVEL), "wait"),
    ]
    return {"right": right}

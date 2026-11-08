"""Track geometry for the passing-loop fixture. Pure Python, shared by task.py, run.py and scoring.py.

World frame as in yam_arena: x forward from the arm bases, y toward the left arm, z up, metres.
The ball runs from +y (left) to -y (right). Every track piece is a channel between two walls.
"""

import math

TOP = 0.7475  # station worktop
BED_T = 0.012
BED = TOP + BED_T  # running surface
WALL_T = 0.008
WALL_H = 0.022
CHANNEL = 0.036  # clear width between walls
D = CHANNEL / 2 + WALL_T / 2  # channel centreline to wall centreline
BALL_R = 0.014
BALL_MASS = 0.030

MAIN_X = 0.44
LOOP_X = 0.54
Y_START = 0.30  # end wall behind the platform
Y_END = -0.30  # buffer stop at the exit
JUNCTION_A = 0.17  # y where the entry diagonal's centreline leaves the main line's centreline
JUNCTION_B = -0.08  # y where the exit diagonal's centreline leaves the loop's centreline
PIT_END = -0.25
BALL_START = (MAIN_X, 0.270, BED + BALL_R)
GATE_Y = 0.245
BLOCKER = (MAIN_X, 0.0)
BLOCKER_LEN = 0.060

S2 = math.sqrt(0.5)


def _diag(origin, direction, offset_normal, offset):
    """A line parallel to a 45-degree centreline, as (point, direction)."""
    return ((origin[0] + offset_normal[0] * offset, origin[1] + offset_normal[1] * offset), direction)


def _x_cut(line, x):
    (px, py), (dx, dy) = line
    s = (x - px) / dx
    return (x, py + s * dy)


# Entry diagonal: from the main line toward +x and -y. Exit diagonal: from the loop toward -x and -y.
_A_DIR, _B_DIR = (S2, -S2), (-S2, -S2)
A_UP = _diag((MAIN_X, JUNCTION_A), _A_DIR, (S2, S2), D)  # upstream wall
A_DOWN = _diag((MAIN_X, JUNCTION_A), _A_DIR, (S2, S2), -D)  # downstream wall, ends at the frog
B_IN = _diag((LOOP_X, JUNCTION_B), _B_DIR, (-S2, S2), D)  # upstream wall
B_OUT = _diag((LOOP_X, JUNCTION_B), _B_DIR, (-S2, S2), -D)  # downstream wall, starts at the frog

MAIN_LO, MAIN_HI = MAIN_X - D, MAIN_X + D
LOOP_LO, LOOP_HI = LOOP_X - D, LOOP_X + D

A_MOUTH = _x_cut(A_UP, MAIN_HI)  # upstream end of the gap in the main line's far wall
A_FROG = _x_cut(A_DOWN, MAIN_HI)  # blade pivot
B_MOUTH = _x_cut(B_IN, LOOP_LO)
B_FROG = _x_cut(B_OUT, LOOP_LO)
A_LOOP_UP, A_LOOP_DOWN = _x_cut(A_UP, LOOP_HI), _x_cut(A_DOWN, LOOP_LO)
B_MAIN_IN, B_MAIN_OUT = _x_cut(B_IN, MAIN_HI), _x_cut(B_OUT, MAIN_HI)

WALLS = {
    "main_near": ((MAIN_LO, Y_START), (MAIN_LO, Y_END)),
    "main_far_1": ((MAIN_HI, Y_START), A_MOUTH),
    "main_far_2": (A_FROG, B_MAIN_IN),
    "main_far_3": (B_MAIN_OUT, (MAIN_HI, Y_END)),
    "entry_up": (A_MOUTH, A_LOOP_UP),
    "entry_down": (A_FROG, A_LOOP_DOWN),
    "loop_far": (A_LOOP_UP, (LOOP_HI, PIT_END)),
    "loop_near_1": (A_LOOP_DOWN, B_MOUTH),
    "loop_near_2": (B_FROG, (LOOP_LO, PIT_END)),
    "exit_up": (B_MOUTH, B_MAIN_IN),
    "exit_down": (B_FROG, B_MAIN_OUT),
    "start_end": ((MAIN_LO, Y_START), (MAIN_HI, Y_START)),
    "exit_buffer": ((MAIN_LO, Y_END), (MAIN_HI, Y_END)),
    "pit_end": ((LOOP_LO, PIT_END), (LOOP_HI, PIT_END)),
}

# Blades: flappers hinged at the frog, pointing upstream (+y). Yaw 0 lies in the wall gap.
BLADE_LEN = A_MOUTH[1] - A_FROG[1] + 0.002
BLADE_THROW = math.asin((2 * D - WALL_T / 2) / BLADE_LEN)
A_NORMAL, A_REVERSE = 0.0, BLADE_THROW  # turned toward -x sends the ball into the loop
B_NORMAL, B_REVERSE = -BLADE_THROW, 0.0  # turned toward +x sends the ball back to the main line

# Points lever, worked by the right arm. It leans away from the arms when normal and toward them when pulled.
LEVER_PIVOT = (0.275, -0.130, TOP + 0.040)
LEVER_ARM = 0.100  # pivot to the handle's centre
LEVER_NORMAL, LEVER_REVERSE = math.radians(22.0), math.radians(-22.0)  # its two stops; it falls onto either

# Timing and the ball's drive.
RELEASE_S = 5.0
BALL_SPEED = 0.07  # m/s on the straight
DRAG = 0.10  # N s/m; with the drive force, a 0.3 s approach to the running speed

# Scoring regions.
# The ball is on a junction while its centre is within its radius plus 5 mm of the area the blade sweeps.
ZONE_PAD = BALL_R + 0.005
_A_TIP_X = A_FROG[0] - BLADE_LEN * math.sin(BLADE_THROW)
_B_TIP_X = B_FROG[0] + BLADE_LEN * math.sin(BLADE_THROW)
ZONE_A = (_A_TIP_X - ZONE_PAD, A_FROG[0] + ZONE_PAD, A_FROG[1] - ZONE_PAD, A_FROG[1] + BLADE_LEN + ZONE_PAD)
ZONE_B = (B_FROG[0] - ZONE_PAD, _B_TIP_X + ZONE_PAD, B_FROG[1] - ZONE_PAD, B_FROG[1] + BLADE_LEN + ZONE_PAD)
LOOP_ENTRY_X = MAIN_HI + 0.020  # ball centre past this has left the main line through junction A
PIT_Y = B_FROG[1] - 0.020  # ball centre past this on the loop is in the sand drag
EXIT_Y = B_MAIN_OUT[1] - 0.060  # ball centre past this on the main line has merged
KEEP_OUT = (0.360, 0.640, -0.340, 0.340, TOP + 0.120)  # x0, x1, y0, y1, z max for either gripper TCP

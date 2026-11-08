"""A model railway with a passing loop, on a grey board in front of the arms. The blue ball is the train: it
waits behind the yellow gate at the left end of the main line, and the gate lifts 5 s after the start. A red
wagon stands on the main line between the two junctions, so the ball has to go round the loop. The black
points lever with the white handle, in front of the right arm, works both junctions. Pulled toward the arms,
it turns the left junction into the loop and the loop's far end into the yellow sand drag. Pushed back to
normal, the left junction runs straight and the loop's far end leads back onto the main line.
"""

from dataclasses import dataclass
import math

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box

import layout as L

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
INSTRUCTION = (
    "The blue ball is a train, and the red wagon is stopped on the main line. Send the ball round the loop "
    "and back onto the main line: pull the black points lever toward you before the ball reaches the first "
    "junction, then push it back once the ball is past that junction and before it reaches the end of the "
    "loop. Never move the lever while the ball is on a junction, and do not touch the track or the ball."
)

BOARD = (0.55, 0.57, 0.58)
WALL = (0.90, 0.91, 0.90)
STEEL = (0.70, 0.72, 0.75)
BLACK = (0.08, 0.08, 0.09)
WHITE = (0.95, 0.95, 0.95)
BLUE = (0.10, 0.36, 0.86)
RED = (0.82, 0.10, 0.10)
YELLOW = (0.98, 0.78, 0.10)
SAND = (0.86, 0.72, 0.42)
DARK = (0.20, 0.22, 0.25)


@dataclass(frozen=True)
class Part:
    name: str
    center: tuple
    size: tuple
    color: tuple
    collides: bool = True
    yaw: float = 0.0
    pitch: float = 0.0  # about the body's y axis, through the body origin
    shape: str = "box"  # "box", or "sphere" with size[0] its diameter


@dataclass(frozen=True)
class Guide:
    kind: str  # "prismatic" or "revolute"
    axis: str
    limits: tuple  # metres, or radians for a revolute guide


@dataclass(frozen=True)
class Body:
    gravity: bool = False
    friction: tuple = (0.4, 0.25)
    damping: tuple = (0.0, 0.0)  # linear, angular


OBJECTS = []
PARTS = {}
GUIDES = {}
BODIES = {}


def body(name, pos, parts, mass=None, guide=None, physics=Body(), yaw=0.0):
    # Box is the Arena registry entry; run.py authors these compound colliders.
    OBJECTS.append(Box(name=name, pos=pos, size=parts[0].size, color=parts[0].color, mass=mass, yaw=yaw))
    PARTS[name] = parts
    BODIES[name] = physics
    if guide is not None:
        GUIDES[name] = guide


def wall(name, p0, p1, height=L.WALL_H, color=WALL):
    (x0, y0), (x1, y1) = p0, p1
    length = math.hypot(x1 - x0, y1 - y0) + L.WALL_T
    return Part(name, ((x0 + x1) / 2, (y0 + y1) / 2, L.BED_T + height / 2), (length, L.WALL_T, height), color,
                yaw=math.atan2(y1 - y0, x1 - x0))


def sleepers(y0, y1, x, prefix):
    count = int(abs(y1 - y0) / 0.05)
    return [Part(f"{prefix}_{i}", (x, y0 - math.copysign(0.025 + 0.05 * i, y0 - y1), L.BED_T + 0.0004),
                 (0.060, 0.010, 0.0008), DARK, False) for i in range(count)]


bed_x = (L.MAIN_LO - 0.035 + L.LOOP_HI + 0.035) / 2
body("track", (0.0, 0.0, TOP), [
    Part("bed", (bed_x, 0.0, L.BED_T / 2), (L.LOOP_HI - L.MAIN_LO + 0.07, L.Y_START - L.Y_END + 0.06, L.BED_T),
         BOARD),
    *[wall(name, p0, p1) for name, (p0, p1) in L.WALLS.items()],
    Part("sand_drag", (L.LOOP_X, (L.B_FROG[1] - 0.02 + L.PIT_END) / 2, L.BED_T + 0.0005),
         (L.CHANNEL, L.B_FROG[1] - 0.02 - L.PIT_END, 0.001), SAND, False),
    Part("exit_line", (L.MAIN_X, L.EXIT_Y, L.BED_T + 0.0009), (L.CHANNEL, 0.004, 0.0008), WHITE, False),
    Part("buffer_beam", (L.MAIN_X, L.Y_END + 0.008, L.BED_T + 0.014), (0.046, 0.006, 0.010), RED, False),
    Part("platform", (L.MAIN_X - 0.050, (L.Y_START + L.GATE_Y) / 2, L.BED_T + 0.004), (0.050, 0.07, 0.008), WALL),
    *sleepers(L.Y_START, L.Y_END, L.MAIN_X, "main_sleeper"),
    *sleepers(L.A_LOOP_UP[1], L.PIT_END, L.LOOP_X, "loop_sleeper"),
    # The lever frame: a fixed stand with the lever's pivot on top.
    Part("lever_frame", (L.LEVER_PIVOT[0], L.LEVER_PIVOT[1], 0.017), (0.080, 0.060, 0.034), DARK),
    Part("rod", ((L.LEVER_PIVOT[0] + L.MAIN_LO) / 2 + 0.01, L.LEVER_PIVOT[1], 0.004),
         (L.MAIN_LO - L.LEVER_PIVOT[0] - 0.02, 0.008, 0.008), STEEL, False),
])

body("blocker", (L.BLOCKER[0], L.BLOCKER[1], L.BED + 0.020), [
    Part("wagon", (0.0, 0.0, 0.0), (0.030, L.BLOCKER_LEN, 0.032), RED),
])

for name, frog, start in (("blade_a", L.A_FROG, L.A_NORMAL), ("blade_b", L.B_FROG, L.B_NORMAL)):
    # Hinged at the frog; the blade runs upstream (+y) from its hinge. Its world yaw is the joint angle.
    body(name, (frog[0], frog[1], L.BED + 0.0015 + 0.010), [
        Part("blade", (0.0, L.BLADE_LEN / 2, 0.0), (L.WALL_T * 0.75, L.BLADE_LEN, 0.020), STEEL),
        Part("hinge", (0.0, 0.0, 0.0), (0.009, 0.009, 0.020), DARK),
    ], 0.020, Guide("revolute", "Z", (-L.BLADE_THROW - 0.05, L.BLADE_THROW + 0.05)), yaw=start)


def leaning(z):
    """A point on the lever's axis, z above the pivot, with the lever at normal (leaning away from the arms)."""
    return (z * math.sin(L.LEVER_NORMAL), 0.0, z * math.cos(L.LEVER_NORMAL))


# The lever is authored at normal, so its joint angle is the lever angle minus LEVER_NORMAL.
body("lever", L.LEVER_PIVOT, [
    Part("shaft", leaning(L.LEVER_ARM / 2), (0.014, 0.014, L.LEVER_ARM), BLACK, pitch=L.LEVER_NORMAL),
    Part("handle", leaning(L.LEVER_ARM + 0.010), (0.022, 0.050, 0.026), WHITE, pitch=L.LEVER_NORMAL),
], 0.150, Guide("revolute", "Y", (L.LEVER_REVERSE - L.LEVER_NORMAL, 0.0)), physics=Body(gravity=True))

body("gate", (L.MAIN_X, L.GATE_Y, L.BED + 0.018), [
    Part("bar", (0.0, 0.0, 0.0), (L.CHANNEL - 0.004, 0.008, 0.028), YELLOW),
    Part("post", (0.0, 0.0, 0.030), (0.008, 0.008, 0.032), YELLOW),
], 0.030, Guide("prismatic", "Z", (0.0, 0.045)))

body("ball", L.BALL_START, [
    Part("ball", (0.0, 0.0, 0.0), (2 * L.BALL_R,) * 3, BLUE, shape="sphere"),
], L.BALL_MASS, physics=Body(gravity=True, friction=(0.5, 0.4)))


def demo():
    """The right arm pulls the lever before the ball reaches junction A and restores it while the ball is in
    the loop. The left arm stays parked. PASSING_LOOP_DEMO picks a validation control instead:
    idle, pull_only, early_restore (pushed back while the ball is on junction A) or late_pull."""
    import os

    from yam_arena.constants import PAD_TIP_PAST_TCP

    variant = os.environ.get("PASSING_LOOP_DEMO", "full")
    px, py, pz = L.LEVER_PIVOT
    knob_z = pz + L.LEVER_ARM * math.cos(L.LEVER_NORMAL) + 0.010
    knob_normal = px + L.LEVER_ARM * math.sin(L.LEVER_NORMAL)
    knob_reverse = px + L.LEVER_ARM * math.sin(L.LEVER_REVERSE)
    push_z = knob_z + PAD_TIP_PAST_TCP - 0.008
    travel_z = TOP + 0.200
    far, near = knob_normal + 0.040, knob_reverse - 0.040

    def wp(duration, x, z, label):
        return Waypoint(duration, (x, py, z), FINGER_CLOSED, label)

    first_wait = {"late_pull": 6.5}.get(variant, 0.0)
    restore_wait = {"early_restore": 0.0}.get(variant, 2.0)
    pull = [
        wp(1.6, far, travel_z, "above, beyond the lever handle"),
        wp(1.0 + first_wait, far, push_z, "down behind the handle"),
        wp(1.3, knob_reverse - 0.006, push_z, "pull the lever"),
        wp(0.6, knob_reverse + 0.012, push_z, "ease off the handle"),
        wp(0.8, knob_reverse + 0.012, travel_z, "lift clear"),
    ]
    restore = [
        wp(0.8, near, travel_z, "above, in front of the handle"),
        wp(1.0, near, push_z, "down in front of the handle"),
        wp(restore_wait, near, push_z, "wait for the ball to clear junction A"),
        wp(1.4, knob_normal + 0.006, push_z, "push the lever back"),
        wp(0.6, knob_normal - 0.012, push_z, "ease off the handle"),
        wp(1.0, knob_normal - 0.012, travel_z, "lift clear"),
    ]
    rest = (0.16, -0.27, TOP + 0.17)
    park = [Waypoint(1.2, rest, FINGER_CLOSED, "park low"),
            Waypoint(9.0, rest, FINGER_CLOSED, "watch the ball run home")]
    right = {"idle": park, "pull_only": pull + park}.get(variant, pull + restore + park)
    left = [Waypoint(1.5, (0.16, 0.27, TOP + 0.17), FINGER_CLOSED, "park low, clear of the camera's view")]
    return {"left": left, "right": right}

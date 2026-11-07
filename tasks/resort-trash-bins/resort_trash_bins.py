"""Draft recycling-bin correction task for yam_arena's station scene.

This scene and its scripted demo have not been run in the simulator. The
primitive shapes stand for materials; they do not simulate material sensing.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR_THICKNESS = 0.004
FLOOR_Z = TOP + FLOOR_THICKNESS
INNER = 0.140
WALL = 0.005
WALL_HEIGHT = 0.030
MAX_FLOOR_GAP = 0.008
MAX_TILT_DEG = 15.0

# x points forward; positive y is to the left of the two arm bases.
DESTINATIONS = {
    "glass": (0.28, 0.16),
    "paper": (0.28, 0.0),
    "metal": (0.28, -0.16),
    "other": (0.45, 0.0),
}
BIN_COLORS = {
    "glass": (0.15, 0.40, 0.85),
    "paper": (0.85, 0.65, 0.20),
    "metal": (0.35, 0.35, 0.38),
    "other": (0.65, 0.20, 0.70),
}
SLOTS = ((-0.035, -0.035), (-0.035, 0.035), (0.035, -0.035), (0.035, 0.035))

# name: (category, starting destination, starting slot, shape, dimensions, color)
# Cylinders use (radius, height); boxes use (x, y, z). All dimensions are m.
ITEMS = {
    "glass_correct": ("glass", "glass", 0, "cylinder", (0.018, 0.065), (0.12, 0.65, 0.45)),
    "metal_in_glass": ("metal", "glass", 1, "cylinder", (0.018, 0.065), (0.72, 0.74, 0.78)),
    "paper_in_glass": ("paper", "glass", 2, "box", (0.040, 0.040, 0.065), (0.62, 0.39, 0.19)),
    "other_in_glass": ("other", "glass", 3, "box", (0.040, 0.040, 0.065), (0.85, 0.10, 0.10)),
    "paper_correct": ("paper", "paper", 0, "box", (0.040, 0.040, 0.065), (0.62, 0.39, 0.19)),
    "glass_in_paper": ("glass", "paper", 1, "cylinder", (0.018, 0.065), (0.12, 0.65, 0.45)),
    "metal_in_paper": ("metal", "paper", 2, "cylinder", (0.018, 0.065), (0.72, 0.74, 0.78)),
    "metal_correct": ("metal", "metal", 0, "cylinder", (0.018, 0.065), (0.72, 0.74, 0.78)),
    "glass_in_metal": ("glass", "metal", 1, "cylinder", (0.018, 0.065), (0.12, 0.65, 0.45)),
    "paper_in_metal": ("paper", "metal", 2, "box", (0.040, 0.040, 0.065), (0.62, 0.39, 0.19)),
}


def _slot(destination, slot):
    x, y = DESTINATIONS[destination]
    dx, dy = SLOTS[slot]
    return x + dx, y + dy


def _bin(name, centre, color):
    x, y = centre
    offset = (INNER + WALL) / 2
    z = FLOOR_Z + WALL_HEIGHT / 2
    outer = INNER + 2 * WALL
    return [
        Box(f"{name}_floor", pos=(x, y, TOP + FLOOR_THICKNESS / 2),
            size=(outer, outer, FLOOR_THICKNESS), color=color),
        Box(f"{name}_north", pos=(x, y + offset, z), size=(outer, WALL, WALL_HEIGHT), color=color),
        Box(f"{name}_south", pos=(x, y - offset, z), size=(outer, WALL, WALL_HEIGHT), color=color),
        Box(f"{name}_east", pos=(x + offset, y, z), size=(WALL, INNER, WALL_HEIGHT), color=color),
        Box(f"{name}_west", pos=(x - offset, y, z), size=(WALL, INNER, WALL_HEIGHT), color=color),
    ]


def _item(name, spec):
    _, destination, slot, shape, dimensions, color = spec
    x, y = _slot(destination, slot)
    height = dimensions[-1]
    pos = (x, y, FLOOR_Z + height / 2 + 0.001)
    if shape == "cylinder":
        return Cylinder(name, pos=pos, radius=dimensions[0], height=height, mass=0.10, color=color)
    return Box(name, pos=pos, size=dimensions, mass=0.10, color=color)


OBJECTS = [part for name, centre in DESTINATIONS.items() for part in _bin(name, centre, BIN_COLORS[name])]
OBJECTS += [_item(name, spec) for name, spec in ITEMS.items()]


def _upright(obj):
    x, y, _, _ = (float(v) for v in obj.quat_xyzw)
    axis_z = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return axis_z >= math.cos(math.radians(MAX_TILT_DEG))


def _in_destination(obj, destination):
    """A resting upright object entirely within the opening, on the floor."""
    if not _upright(obj) or not obj.resting():
        return False
    cx, cy = DESTINATIONS[destination]
    low, high = obj.low, obj.high
    # The arena's AABB for cylinders is conservative when the cylinder spins.
    return bool(
        low[0] >= cx - INNER / 2 and high[0] <= cx + INNER / 2
        and low[1] >= cy - INNER / 2 and high[1] <= cy + INNER / 2
        and FLOOR_Z - 0.002 <= low[2] <= FLOOR_Z + MAX_FLOOR_GAP
    )


def score(objects):
    """Each clean destination containing all its assigned objects earns 0.25."""
    total = 0.0
    for destination in DESTINATIONS:
        correct = all(_in_destination(objects[name], destination)
                      for name, spec in ITEMS.items() if spec[0] == destination)
        foreign = any(_in_destination(objects[name], destination)
                      for name, spec in ITEMS.items() if spec[0] != destination)
        if correct and not foreign:
            total += 0.25
    return total


# One empty slot per recycling bin provides a place to break the cyclic swaps.
# Plan entries are (object, destination, destination slot, arm). The other arm
# waits at its parking position. This is an unvalidated candidate demonstration.
MOVES = (
    ("metal_in_glass", "metal", 3, "left"),
    ("glass_in_metal", "glass", 1, "right"),
    ("metal_in_paper", "metal", 1, "right"),
    ("paper_in_metal", "paper", 3, "right"),
    ("paper_in_glass", "paper", 2, "left"),
    ("glass_in_paper", "glass", 2, "left"),
    ("other_in_glass", "other", 0, "left"),
)


def demo():
    """Candidate scripted moves; reachability and grasping are not verified."""
    park = {"left": (0.12, 0.40, TOP + 0.25), "right": (0.12, -0.40, TOP + 0.25)}
    plans = {side: [Waypoint(2.0, pos, FINGER_OPEN, "park")] for side, pos in park.items()}
    grip_z = FLOOR_Z + 0.040
    carry_z = TOP + 0.22
    for name, destination, slot, side in MOVES:
        _, source, source_slot, _, _, _ = ITEMS[name]
        sx, sy = _slot(source, source_slot)
        dx, dy = _slot(destination, slot)
        steps = [
            Waypoint(2.0, (sx, sy, carry_z), FINGER_OPEN, f"{name}: approach"),
            Waypoint(1.5, (sx, sy, grip_z), FINGER_OPEN, f"{name}: descend"),
            Waypoint(0.8, (sx, sy, grip_z), FINGER_CLOSED, f"{name}: grasp"),
            Waypoint(1.5, (sx, sy, carry_z), FINGER_CLOSED, f"{name}: lift"),
            Waypoint(2.0, (dx, dy, carry_z), FINGER_CLOSED, f"{name}: carry"),
            Waypoint(1.5, (dx, dy, grip_z + 0.002), FINGER_CLOSED, f"{name}: lower"),
            Waypoint(0.8, (dx, dy, grip_z + 0.002), FINGER_OPEN, f"{name}: release"),
            Waypoint(1.2, (dx, dy, carry_z), FINGER_OPEN, f"{name}: rise"),
            Waypoint(2.0, park[side], FINGER_OPEN, f"{name}: park"),
        ]
        plans[side].extend(steps)
        other = "right" if side == "left" else "left"
        plans[other].append(Waypoint(sum(w.duration for w in steps), park[other], FINGER_OPEN, "wait"))
    return plans

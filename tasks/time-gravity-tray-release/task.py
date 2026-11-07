"""Stage a ball behind a stop and feed the empty moving sorter pocket from a fixed chute."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from partkit import Slider, on_reset, on_step, part
from release_logic import (
    BALL_RADIUS,
    CHUTE_EXIT,
    CHUTE_START,
    CHUTE_X,
    DT,
    GATE_BASE_GAP,
    GATE_LIFT,
    GATE_THICKNESS,
    GATE_Y,
    PITCH,
    POCKET_OFFSET,
    SHUTTLE_MAX,
    SHUTTLE_MIN,
    SHUTTLE_TRAVEL,
    TRAY_FLOOR,
    TRAY_INNER_X,
    TRAY_INNER_Y,
    TRAY_WALL_TOP,
    TRAY_Y,
    TRIALS,
    GatePose,
    ReleaseScore,
    deck,
    demo_lift_time,
)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Sphere

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = (
    "Use the two gates to send the yellow ball into the empty moving tray. "
    "Let the ball stop at the second gate before releasing it."
)

TRIAL = TRIALS[os.environ.get("SORTER_TRIAL", "A")]
STEEL = (0.24, 0.30, 0.36)
TRACK = (0.49, 0.56, 0.61)
GUARD = (0.72, 0.77, 0.79)
TRAY = (0.10, 0.42, 0.70)
LINER = (0.08, 0.16, 0.23)
RIB_HEIGHT = 0.003
CARGO = (0.78, 0.16, 0.10)
GATE_COLORS = ((0.10, 0.62, 0.53), (0.96, 0.46, 0.12))
YELLOW = (1.00, 0.86, 0.10)
PITCH_QUAT = (math.cos(PITCH / 2), math.sin(PITCH / 2), 0.0, 0.0)
HANDLE_CENTER_Z = 0.056
BLADE_WIDTH = 0.092
GUIDE_X = 0.044
SLOT_HALF_Y = 0.016


def box(x, y, z, dx, dy, dz, color):
    return ((x, y, z), (dx, dy, dz), color)


def pitched_box(x, y0, y1, z_offset, width, thickness, color):
    y = (y0 + y1) / 2
    return (
        *box(
            x,
            y,
            deck(y) + z_offset,
            width,
            (y1 - y0) / math.cos(PITCH),
            thickness,
            color,
        ),
        PITCH_QUAT,
    )


def chute_boxes():
    # The floor's upper face follows deck(y). The upper guard has a 22 mm sight slit.
    boxes = [
        pitched_box(
            CHUTE_X,
            CHUTE_EXIT,
            CHUTE_START,
            -0.008 / math.cos(PITCH),
            0.124,
            0.016,
            TRACK,
        ),
        box(
            CHUTE_X,
            CHUTE_START - 0.004,
            deck(CHUTE_START) + 0.024,
            0.122,
            0.008,
            0.052,
            STEEL,
        ),
    ]
    segments = (
        (CHUTE_EXIT, GATE_Y[1] - SLOT_HALF_Y),
        (GATE_Y[1] + SLOT_HALF_Y, GATE_Y[0] - SLOT_HALF_Y),
        (GATE_Y[0] + SLOT_HALF_Y, CHUTE_START),
    )
    for side in (-1, 1):
        boxes.append(
            pitched_box(
                CHUTE_X + side * 0.062,
                CHUTE_EXIT,
                CHUTE_START,
                0.021,
                0.007,
                0.043,
                STEEL,
            )
        )
        for y0, y1 in segments:
            boxes.append(
                pitched_box(CHUTE_X + side * 0.032, y0, y1, 0.023, 0.008, 0.045, TRACK)
            )
            boxes.append(
                pitched_box(CHUTE_X + side * 0.035, y0, y1, 0.052, 0.048, 0.006, GUARD)
            )
        for y, color in zip(GATE_Y, GATE_COLORS, strict=True):
            x = CHUTE_X + side * GUIDE_X
            seat_z = deck(y) + GATE_BASE_GAP
            # The downhill cheeks bear the ball load; the gate extracts in an open vertical slot.
            boxes.append(box(x, y, seat_z - 0.004, 0.023, 0.026, 0.008, color))
            boxes.append(box(x, y - 0.015, seat_z + 0.014, 0.023, 0.012, 0.030, color))
            boxes.append(box(x, y + 0.024, seat_z + 0.014, 0.023, 0.009, 0.030, color))
    for y in (-0.340, 0.015, 0.216):
        height = deck(y) - 0.013
        boxes.append(box(CHUTE_X, y, height / 2, 0.106, 0.034, height, STEEL))
    return boxes


def make_gate(index, name):
    y = GATE_Y[index]
    color = GATE_COLORS[index]
    return part(
        name,
        (CHUTE_X, y, TOP + deck(y) + GATE_BASE_GAP + 0.001),
        (
            box(0.0, 0.0, 0.022, BLADE_WIDTH, GATE_THICKNESS, 0.044, color),
            box(0.0, 0.0, HANDLE_CENTER_Z, 0.029, 0.022, 0.030, color),
        ),
        mass=0.095,
        friction=0.6,
    )


def pocket_boxes():
    outer_x = TRAY_INNER_X + 0.005
    outer_y = TRAY_INNER_Y + 0.005
    wall_h = TRAY_WALL_TOP - TRAY_FLOOR
    wall_z = TRAY_FLOOR + wall_h / 2
    boxes = []
    for pocket in (-1, 1):
        x = -TRIAL.direction * pocket * POCKET_OFFSET
        boxes.append(
            box(x, 0, TRAY_FLOOR - 0.004, 2 * outer_x, 2 * outer_y, 0.008, TRAY)
        )
        for side in (-1, 1):
            end_h = 0.010 if side == -TRIAL.direction else wall_h
            boxes.extend(
                (
                    box(
                        x + side * (TRAY_INNER_X + 0.0025),
                        0,
                        wall_z,
                        0.005,
                        2 * outer_y,
                        wall_h,
                        TRAY,
                    ),
                    box(
                        x,
                        side * (TRAY_INNER_Y + 0.0025),
                        TRAY_FLOOR + end_h / 2,
                        2 * TRAY_INNER_X,
                        0.005,
                        end_h,
                        TRAY,
                    ),
                )
            )
        # Crossed ribs interrupt rolling in both directions while keeping the floor height measurable.
        rib_z = TRAY_FLOOR + RIB_HEIGHT / 2
        for rib in (-1, 0, 1):
            boxes.append(
                box(
                    x + rib * 0.024,
                    0,
                    rib_z,
                    0.004,
                    2 * TRAY_INNER_Y,
                    RIB_HEIGHT,
                    LINER,
                )
            )
        for rib in (-2, -1, 0, 1, 2):
            boxes.append(
                box(x, rib * 0.024, rib_z, 2 * TRAY_INNER_X, 0.004, RIB_HEIGHT, LINER)
            )
        if pocket == TRIAL.occupied:
            # This inert load marker is fixed to the tray in the simplified sorter.
            boxes.append(
                box(x, 0, TRAY_FLOOR + RIB_HEIGHT + 0.022, 0.066, 0.066, 0.044, CARGO)
            )
    boxes.append(box(0, 0, 0.010, 0.240, 0.022, 0.008, STEEL))
    return tuple(boxes)


FIRST_GATE = make_gate(0, "feed_gate")
SECOND_GATE = make_gate(1, "release_gate")
ball_y = GATE_Y[0] + GATE_THICKNESS / 2 + BALL_RADIUS + 0.004
BALL = Sphere(
    "yellow_ball",
    pos=(CHUTE_X, ball_y, TOP + deck(ball_y) + BALL_RADIUS / math.cos(PITCH) + 0.001),
    radius=BALL_RADIUS,
    mass=0.030,
    color=YELLOW,
)

CARRIER = part(
    "sorter_shuttle",
    (TRIAL.root_x, TRAY_Y, TOP),
    (box(0, 0.100, 0.004, 0.036, 0.036, 0.008, STEEL),),
    mass=2.0,
    yaw=math.pi if TRIAL.direction > 0 else 0.0,
    fixed_root=True,
    sliders=(
        Slider(
            "feed",
            rest=(0.0, 0.0, 0.0),
            boxes=pocket_boxes(),
            travel=SHUTTLE_TRAVEL,
            axis="X",
            rate=4200.0,
            damping=95.0,
            mass=0.60,
            friction=0.70,
        ),
    ),
)

rail_x = (SHUTTLE_MIN + SHUTTLE_MAX) / 2
rail_length = SHUTTLE_TRAVEL + 2 * (POCKET_OFFSET + TRAY_INNER_X + 0.008)
OBJECTS = [
    part("gravity_chute", (0.0, 0.0, TOP), chute_boxes(), mass=None),
    part(
        "sorter_rail",
        (rail_x, TRAY_Y, TOP),
        (
            box(0, 0, 0.002, rail_length, 0.155, 0.004, STEEL),
            box(0, -0.073, 0.006, rail_length, 0.005, 0.004, TRACK),
            box(0, 0.073, 0.006, rail_length, 0.005, 0.004, TRACK),
        ),
        mass=None,
    ),
    FIRST_GATE,
    SECOND_GATE,
    BALL,
    CARRIER,
]

_trial_score = ReleaseScore(TRIAL)
_elapsed = 0.0


def _drive(environment, time_s):
    asset = environment.unwrapped.scene[CARRIER.name]
    command = asset.actuators.target_command
    shape = asset.data.default_joint_pos.torch
    current = TRIAL.joint_target(time_s)
    following = TRIAL.joint_target(time_s + DT)
    command.set_position_index(value=shape.clone().fill_(following))
    command.set_velocity_index(value=shape.clone().fill_((following - current) / DT))


@on_reset
def _reset(environment):
    global _elapsed, _trial_score
    if not math.isclose(environment.unwrapped.step_dt, DT, abs_tol=1e-5):
        raise ValueError("This task requires -E control_hz=20")
    _elapsed = 0.0
    _trial_score = ReleaseScore(TRIAL)
    _drive(environment, _elapsed)


@on_step
def _step(environment):
    global _elapsed
    _elapsed += DT
    _drive(environment, _elapsed)


def _gate_pose(state):
    rotation = state.rotation()
    x, y, z = map(float, state.pos)
    return GatePose((x, y, z - TOP), float(rotation[2, 2]), float(rotation[0, 0]))


def score(objects):
    x, y, z = map(float, objects[BALL.name].pos)
    gates = tuple(_gate_pose(objects[gate.name]) for gate in (FIRST_GATE, SECOND_GATE))
    shuttle = objects[CARRIER.name]
    shuttle_x = float(shuttle.pos[0]) - TRIAL.direction * shuttle.sliders["feed"]
    return _trial_score.step((x, y, z - TOP), gates, shuttle_x)


def _grip_gate(gate):
    x, y, z = gate.pos
    grasp = z - 0.001 + HANDLE_CENTER_Z + PAD_TIP_PAST_TCP
    high = grasp + GATE_LIFT
    yaw = math.pi / 2
    approach = [
        Waypoint(1.40, (x, y, high), FINGER_OPEN, "approach gate", yaw),
        Waypoint(1.00, (x, y, grasp), FINGER_OPEN, "lower open jaws", yaw),
        Waypoint(0.75, (x, y, grasp), FINGER_CLOSED, "grasp gate handle", yaw),
    ]
    return approach, Waypoint(
        0.65, (x, y, high), FINGER_CLOSED, "lift gate vertically", yaw
    )


def demo():
    mode = os.environ.get("SORTER_DEMO", "timed")
    if mode == "idle":
        return {}
    if mode not in ("timed", "staged", "aligned", "early", "occupied", "fixed"):
        raise ValueError(f"Unknown SORTER_DEMO {mode!r}")

    left, lift_left = _grip_gate(FIRST_GATE)
    left.append(lift_left)
    if mode == "staged":
        return {"left": left}

    right, lift_right = _grip_gate(SECOND_GATE)
    wait = demo_lift_time(TRIAL, mode) - sum(step.duration for step in right)
    if wait > 0:
        hold = right[-1]
        right.append(
            Waypoint(
                wait, hold.pos, hold.finger, "hold at gate until release", hold.yaw
            )
        )
    right.extend(
        (
            lift_right,
            Waypoint(
                2.0,
                lift_right.pos,
                lift_right.finger,
                "keep the chute clear while the ball settles",
                lift_right.yaw,
            ),
        )
    )
    return {"left": left, "right": right}

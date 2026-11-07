"""Geometry, deterministic shuttle timing and score rules (no simulator imports)."""

from __future__ import annotations

import math
from dataclasses import dataclass

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
BALL_RADIUS = 0.020
CHUTE_X = 0.225
CHUTE_START = 0.252
CHUTE_EXIT = -0.415
EXIT_HEIGHT = 0.042
PITCH = math.radians(5.5)
GATE_Y = (0.190, -0.090)
GATE_THICKNESS = 0.014
GATE_BASE_GAP = 0.003
GATE_LIFT = 0.050
GATE_OPEN = 0.044
TRAY_Y = -0.490
TRAY_FLOOR = 0.016
TRAY_WALL_TOP = 0.046
TRAY_INNER_X = 0.048
TRAY_INNER_Y = 0.061
POCKET_OFFSET = 0.067
SHUTTLE_MIN = 0.080
SHUTTLE_MAX = 0.390
SHUTTLE_TRAVEL = SHUTTLE_MAX - SHUTTLE_MIN
TURN_DWELL_S = 1.0

# The demo uses an initial rolling-time estimate. The scorer measures the ball and tray poses.
ROLL_AND_DROP_S = 1.08
LIFT_TO_RELEASE_S = 0.43
EARLIEST_LIFT_S = 5.75
Vec3 = tuple[float, float, float]


def deck(y: float) -> float:
    """Track height at y, measured vertically from the table."""
    return EXIT_HEIGHT + (y - CHUTE_EXIT) * math.tan(PITCH)


@dataclass(frozen=True)
class Trial:
    speed: float
    direction: int
    occupied: int
    delay: float = 0.0

    @property
    def root_x(self) -> float:
        return SHUTTLE_MIN if self.direction > 0 else SHUTTLE_MAX

    @property
    def empty(self) -> int:
        return -self.occupied

    def joint_target(self, time_s: float) -> float:
        """Travel from the home stop and back along the shuttle's local negative x axis."""
        traverse = SHUTTLE_TRAVEL / self.speed
        leg = traverse + TURN_DWELL_S
        phase = max(0.0, time_s - self.delay) % (2.0 * leg)
        outbound = min(phase, traverse)
        inbound = min(max(0.0, phase - leg), traverse)
        return -(outbound - inbound) * self.speed

    def world_x(self, joint_position: float) -> float:
        return self.root_x - self.direction * joint_position

    def next_alignment(self, pocket: int, earliest: float) -> float:
        """Next time the chosen pocket centre crosses the chute centre."""
        shuttle_x = CHUTE_X - pocket * POCKET_OFFSET
        distance = abs(shuttle_x - self.root_x)
        cycle = 2.0 * (SHUTTLE_TRAVEL / self.speed + TURN_DWELL_S)
        offsets = (
            distance / self.speed,
            (2.0 * SHUTTLE_TRAVEL - distance) / self.speed + TURN_DWELL_S,
        )
        return min(
            first + cycle * max(0, math.ceil((earliest - first) / cycle))
            for first in (self.delay + offset for offset in offsets)
        )


TRIALS = {
    "A": Trial(0.100, -1, +1),
    "B": Trial(0.100, -1, -1),
    "C": Trial(0.080, +1, +1, 0.45),
    "D": Trial(0.120, +1, -1, 0.80),
}


def demo_lift_time(trial: Trial, mode: str) -> float:
    if mode == "early":
        return 3.20
    if mode == "fixed":
        trial = TRIALS["A"]
    if mode == "aligned":
        return trial.next_alignment(trial.empty, EARLIEST_LIFT_S)
    pocket = trial.occupied if mode == "occupied" else trial.empty
    lead = ROLL_AND_DROP_S + LIFT_TO_RELEASE_S
    return trial.next_alignment(pocket, EARLIEST_LIFT_S + lead) - lead


@dataclass(frozen=True)
class GatePose:
    pos: Vec3
    vertical: float
    across: float

    def in_slot(self, gate: int) -> bool:
        x, y, _ = self.pos
        return (
            abs(x - CHUTE_X) <= 0.013
            and abs(y - GATE_Y[gate]) <= 0.019
            and self.vertical >= 0.97
            and self.across >= 0.96
        )

    def lift(self, gate: int) -> float:
        return self.pos[2] - deck(GATE_Y[gate]) - GATE_BASE_GAP

    def seated(self, gate: int) -> bool:
        return self.in_slot(gate) and -0.007 <= self.lift(gate) <= 0.009


class ReleaseScore:
    """Score one trial from consecutive control-step poses in the table frame."""

    def __init__(self, trial: Trial):
        self.trial = trial
        self.opened = [False, False]
        self.staged_steps = 0
        self.staged = False
        self.exited = False
        self.transit_steps = 0
        self.drop_steps = 0
        self.landed = False
        self.settled_steps = 0
        self.failed = False
        self.previous_ball: Vec3 | None = None
        self.previous_shuttle: float | None = None

    def _fail(self) -> float:
        self.failed = True
        return 0.0

    def step(
        self, ball: Vec3, gates: tuple[GatePose, GatePose], shuttle_x: float
    ) -> float:
        if self.failed:
            return 0.0
        if not all(math.isfinite(value) for value in (*ball, shuttle_x)):
            return self._fail()
        for gate in gates:
            if not all(
                math.isfinite(value)
                for value in (*gate.pos, gate.vertical, gate.across)
            ):
                return self._fail()

        previous_ball = self.previous_ball
        previous_shuttle = self.previous_shuttle
        self.previous_ball = ball
        self.previous_shuttle = shuttle_x
        shuttle_delta = (
            0.0 if previous_shuttle is None else shuttle_x - previous_shuttle
        )
        x, y, z = ball

        # Gate 2 must remain physically seated until the dwell at its face has been observed.
        if not self.staged and not gates[1].seated(1):
            return self._fail()
        for index, gate in enumerate(gates):
            if not self.opened[index]:
                if not gate.in_slot(index) or gate.lift(index) < -0.007:
                    return self._fail()
                if gate.lift(index) >= GATE_OPEN:
                    self.opened[index] = True
            if not self.opened[index] and y < GATE_Y[index] - 0.008:
                return self._fail()

        if not self.exited:
            if self.opened[1]:
                self.transit_steps += 1
                if self.transit_steps > round(1.40 / DT):
                    return self._fail()
            # The guarded track is the only allowed way to the outlet.
            height_error = z - deck(y) - BALL_RADIUS / math.cos(PITCH)
            if (
                abs(x - CHUTE_X) > 0.025
                or y > CHUTE_START
                or not -0.020 <= height_error <= 0.016
            ):
                return self._fail()
            if previous_ball is not None and math.dist(ball, previous_ball) > 0.070:
                return self._fail()
            if y <= CHUTE_EXIT + 0.006:
                if not self.staged or self.transit_steps < round(0.50 / DT):
                    return self._fail()
                self.exited = True

        if not self.staged:
            still = (
                previous_ball is not None
                and math.dist(ball, previous_ball) <= 0.025 * DT
            )
            against_stop = GATE_Y[1] + 0.019 <= y <= GATE_Y[1] + 0.045
            self.staged_steps = (
                self.staged_steps + 1
                if self.opened[0] and still and against_stop
                else 0
            )
            self.staged = self.staged_steps >= round(0.5 / DT)

        if not self.exited:
            held = GATE_Y[1] + 0.019 <= y <= GATE_Y[1] + 0.045
            return (
                0.25
                if self.staged and not self.opened[1] and gates[1].seated(1) and held
                else 0.0
            )

        empty_x = shuttle_x + self.trial.empty * POCKET_OFFSET
        occupied_x = shuttle_x + self.trial.occupied * POCKET_OFFSET
        in_tray_y = abs(y - TRAY_Y) <= TRAY_INNER_Y - BALL_RADIUS + 0.005
        in_empty = abs(x - empty_x) <= TRAY_INNER_X - BALL_RADIUS + 0.005 and in_tray_y
        in_occupied = abs(x - occupied_x) <= TRAY_INNER_X and in_tray_y

        if not self.landed:
            self.drop_steps += 1
            if (
                self.drop_steps > round(0.35 / DT)
                or not TRAY_Y - TRAY_INNER_Y <= y <= CHUTE_EXIT + 0.018
                or abs(x - CHUTE_X) > 0.040
                or z > EXIT_HEIGHT + BALL_RADIUS + 0.012
            ):
                return self._fail()
            if in_occupied and z <= TRAY_WALL_TOP + 0.008:
                return self._fail()
            if z <= TRAY_FLOOR + BALL_RADIUS + 0.008:
                if not in_empty or abs(shuttle_delta) < 0.040 * DT:
                    return self._fail()
                self.landed = True

        if not self.landed:
            return 0.0
        if (
            not in_empty
            or not TRAY_FLOOR + BALL_RADIUS - 0.009 <= z <= TRAY_WALL_TOP + BALL_RADIUS
        ):
            return self._fail()

        relative_motion = (
            (
                x - previous_ball[0] - shuttle_delta,
                y - previous_ball[1],
                z - previous_ball[2],
            )
            if previous_ball is not None
            else (0.0, 0.0, 0.0)
        )
        on_floor = abs(z - (TRAY_FLOOR + BALL_RADIUS)) <= 0.008
        if on_floor and math.hypot(*relative_motion) <= 0.050 * DT:
            self.settled_steps += 1
        else:
            self.settled_steps = 0
        return 1.0 if self.settled_steps >= round(1.0 / DT) else 0.0

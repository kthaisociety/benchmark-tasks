"""Score one run of the ball through the passing loop. Pure Python; fed one Sample per control step."""

from dataclasses import dataclass, field
import math

import layout as L

FULL, LOOP = 1.0, 0.5
SETTLED = 0.1  # lever travel fraction from an end that still counts as fully over (locked)
MOVED = 0.10  # lever travel fraction that counts as a throw while the ball is on a junction
TRACK_Z = (L.BED + L.BALL_R - 0.008, L.BED + L.BALL_R + 0.012)


@dataclass(frozen=True)
class Sample:
    ball: tuple  # ball centre, world (m)
    lever: float  # lever angle (rad): LEVER_NORMAL when normal, LEVER_REVERSE when pulled
    tcp: tuple  # each gripper TCP, world (m)


def lever_travel(angle: float) -> float:
    """0 when normal, 1 when fully pulled."""
    return min(1.0, max(0.0, (L.LEVER_NORMAL - angle) / (L.LEVER_NORMAL - L.LEVER_REVERSE)))


def in_box(xy, box) -> bool:
    x0, x1, y0, y1 = box
    return x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1


@dataclass
class Run:
    entered_loop: bool = False
    merged: bool = False
    in_pit: bool = False
    fault: str | None = None
    zone_lever: dict = field(default_factory=dict)  # junction -> lever travel when the ball came onto it

    def step(self, sample: Sample) -> float:
        if self.fault or self.merged:
            return self.score
        x, y, z = sample.ball
        values = (*sample.ball, sample.lever, *(v for p in sample.tcp for v in p))
        if not all(math.isfinite(v) for v in values):
            self.fault = "non-finite state"
            return self.score
        x0, x1, y0, y1, zmax = L.KEEP_OUT
        if any(x0 <= px <= x1 and y0 <= py <= y1 and pz <= zmax for px, py, pz in sample.tcp):
            self.fault = "gripper over the track"
            return self.score
        if not TRACK_Z[0] <= z <= TRACK_Z[1]:
            self.fault = "ball left the track"
            return self.score

        travel = lever_travel(sample.lever)
        for name, box in (("A", L.ZONE_A), ("B", L.ZONE_B)):
            if in_box((x, y), box):
                start = self.zone_lever.setdefault(name, travel)
                if SETTLED < travel < 1 - SETTLED or abs(travel - start) > MOVED:
                    self.fault = f"lever moved with the ball on junction {name}"
                    return self.score
            else:
                self.zone_lever.pop(name, None)

        if x >= L.LOOP_ENTRY_X:
            self.entered_loop = True
        if self.entered_loop and x >= L.LOOP_LO and y <= L.PIT_Y:
            self.in_pit = True
        if self.entered_loop and not self.in_pit and abs(x - L.MAIN_X) <= 0.02 and y <= L.EXIT_Y:
            self.merged = True
        return self.score

    @property
    def score(self) -> float:
        if self.fault:
            return 0.0
        if self.merged:
            return FULL
        return LOOP if self.entered_loop else 0.0

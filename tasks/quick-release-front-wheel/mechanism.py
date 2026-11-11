"""How the quick-release lever behaves, as spring targets for its two joints. Pure Python: task.py applies the targets
after every control step, and test_scoring.py checks them offline.

- Closure c (deg, 0 open .. 90 closed). Under OPEN_DETENT the lever springs back fully open. Past CLOSE_DETENT the cam
  goes over centre and the lever snaps closed. In between it stays where it is left: its spring's rest point trails
  the lever by up to SLIP, which works like friction (it holds the lever against its own weight, about 0.002 N m,
  and gives way to about 0.2 N at the paddle).
- Aim a (deg): the skewer's turn about the axle, held the same way by friction while the lever is open. Once the
  lever is closed (c >= CLOSED) the skewer is clamped: the rest point stays where it was when the lever closed.
"""

import math

import layout as L

CLOSED = 80.0  # closure counted as closed (it snaps to 90 past CLOSE_DETENT)


def _trail(anchor, value, slip):
    if anchor is None:
        return value
    if value - anchor > slip:
        return value - slip
    if anchor - value > slip:
        return value + slip
    return anchor


class Lever:
    def __init__(self):
        self.cam_anchor = None
        self.aim_anchor = 0.0
        self.clamp = None

    def targets(self, cam_q, aim_q):
        """(cam target, aim target), joint units (rad), from the joints' positions (rad)."""
        c, a = -math.degrees(cam_q), math.degrees(aim_q)
        if c >= L.CLOSE_DETENT:
            self.cam_anchor, cam = None, 90.0
        elif c <= L.OPEN_DETENT:
            self.cam_anchor, cam = None, 0.0
        else:
            self.cam_anchor = cam = _trail(self.cam_anchor, c, L.SLIP)
        if c >= CLOSED:
            if self.clamp is None:
                self.clamp = a
            aim = self.aim_anchor = self.clamp
        else:
            self.clamp = None
            aim = self.aim_anchor = _trail(self.aim_anchor, a, L.SLIP)
        return -math.radians(cam), math.radians(aim)

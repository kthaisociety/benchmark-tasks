"""Checks the let-go test in task.py without the simulator: poses both YAM arms by the URDF's kinematics (the same IK
the scripted demo uses) around the open bottle standing on the coaster, and prints, for each case, the gap from the
nearest part of either arm to the bottle and whether the bottle counts as let go. Needs only yam_arena and NumPy.

    python check_release.py
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import task as T  # noqa: E402
from partkit import PartState  # noqa: E402
from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402
from yam_arena.urdf import forward_kinematics  # noqa: E402

BASE = np.array([T.COASTER[0], T.COASTER[1], T.TOP + T.COASTER_H])  # the bottle standing on the coaster
BOTTLE = PartState(pos=BASE, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=np.zeros(3), high=np.zeros(3))
GRIP_POINT = BASE + (0.0, 0.0, T.GRIP_UP)  # where the demo grips the lower neck


def arm(side: str, tcp=None, finger=FINGER_OPEN, yaw=0.0):
    """Link poses (position, quat_xyzw) of one arm with its TCP at ``tcp`` (a straight-down grip turned ``yaw``), or
    at the ready pose; and where its TCP is."""
    base = np.asarray(station.BASES[side])
    q = np.array(READY, dtype=float)
    if tcp is not None:
        q, miss, _ = ik.solve(tcp, ik.matrix_to_quat_xyzw(grip_rotation(yaw)), base_pos=base, iters=400)
        assert miss < 0.002, f"{side} arm can't reach {np.round(tcp, 3)} ({miss * 1000:.1f} mm off)"
    poses = forward_kinematics(YAM_URDF, {**dict(zip(ARM_JOINTS, map(float, q))), "joint7": finger,
                                          "joint8": finger})
    links = {n: (poses[n][:3, 3] + base, ik.matrix_to_quat_xyzw(poses[n][:3, :3])) for n in T.ARM_LINKS}
    return links, ik.tcp_pose(q, base)[0]


CASES = [
    ("right gripper closed on the lower neck", "right", GRIP_POINT, FINGER_CLOSED, 0.0),
    ("right gripper half open round the neck", "right", GRIP_POINT, FINGER_OPEN / 2, 0.0),
    ("right gripper fully open round the neck (the demo's let-go)", "right", GRIP_POINT, FINGER_OPEN, 0.0),
    ("left fingertips resting on the body, 7 cm up", "left", BASE + (0.0, 0.056, 0.07), FINGER_CLOSED, math.pi / 2),
    ("left fingertips 6 mm off the body, 7 cm up", "left", BASE + (0.0, 0.064, 0.07), FINGER_CLOSED, math.pi / 2),
    ("left fingertips 2 cm off the body, 7 cm up", "left", BASE + (0.0, 0.08, 0.07), FINGER_CLOSED, math.pi / 2),
]

if __name__ == "__main__":
    T.arm_samples()
    print(f"let go: every part of both arms at least {T.LET_GO * 100:.0f} cm from the bottle "
          f"(the old test: each TCP over 6 cm from the grip point)")
    for label, side, tcp, finger, yaw in CASES:
        moved, tcp_at = arm(side, tcp, finger, yaw)
        other, other_tcp = arm("left" if side == "right" else "right")
        poses = {f"{side}_robot": moved, f"{'left' if side == 'right' else 'right'}_robot": other}
        gap, nearest = T.arms_clearance(poses, BOTTLE)
        old = min(np.linalg.norm(tcp_at - GRIP_POINT), np.linalg.norm(other_tcp - GRIP_POINT)) > 0.06
        away = np.linalg.norm(tcp_at - GRIP_POINT)
        print(f"- {label}: nearest {nearest}, gap {gap * 1000:+.1f} mm, TCP {away * 100:.1f} cm from the grip point"
              f" -> let go: {gap >= T.LET_GO} (old test: {old})")

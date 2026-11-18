"""Checks the let-go test in task.py without the simulator: poses both YAM arms with the URDF's kinematics (the same IK
the scripted demo uses) around the open bottle standing on the coaster, and prints, for each case, the gap from the
nearest part of either arm to the bottle and whether the bottle counts as let go. Needs only yam_arena and NumPy.
test_task.py runs the same cases through the scorer's runtime path (a stand-in for the simulator's arm data).

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
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import DOWN_GRIP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix  # noqa: E402

BASE = np.array([T.COASTER[0], T.COASTER[1], T.TOP + T.COASTER_H])  # the bottle standing on the coaster
BOTTLE = PartState(pos=BASE, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                   low=np.zeros(3), high=np.zeros(3))
DOWN = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP))
SIDE = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.pi / 2) @ DOWN  # straight down, fingers closing along x


def arm_joints(side: str, tcp=None, R=None, finger=FINGER_OPEN) -> tuple[dict[str, float], np.ndarray, np.ndarray]:
    """(joint angles joint1..joint8, base position, TCP position) of one arm with its TCP at ``tcp`` turned ``R``, or
    at the ready pose."""
    base = np.asarray(station.BASES[side], dtype=np.float64)
    q = np.array(READY, dtype=float)
    if tcp is not None:
        q, miss, _ = ik.solve(np.asarray(tcp, float), ik.matrix_to_quat_xyzw(R), base_pos=base, iters=400)
        assert miss < 0.002, f"{side} arm can't reach {np.round(tcp, 3)} ({miss * 1000:.1f} mm off)"
    joints = {**dict(zip(ARM_JOINTS, map(float, q), strict=True)), "joint7": finger, "joint8": finger}
    return joints, base, ik.tcp_pose(q, base)[0]


def cases() -> list[tuple[str, str, tuple, bool]]:
    """(label, arm doing it, arm_joints arguments, let go?) with the bottle on the coaster; the other arm is at its
    ready pose. Fingers that hold the glass are posed with their pads on it, where the simulator's stop."""
    body = BASE + (0.0, 0.0, T.GRIP_UP)
    neck = BASE + (0.0, 0.0, T.NECK_GRIP_UP)
    r0 = T.grip_R(0.0)
    return [
        ("right hand holding the lower neck from above", "right", (neck, DOWN, -T.NECK_GRIP_R), False),
        ("right hand holding the body (the demo's grip)", "right", (body, r0, -T.BODY_R), False),
        ("right hand's open pad resting on the body (touch_body)", "right",
         (body + (0.0, 0.020, 0.0), r0, FINGER_OPEN), False),
        ("right fingers just opened round the body (a fingertip still within 1 cm)", "right", (body, r0, FINGER_OPEN),
         False),
        ("right hand backed off 8 cm along its approach (the demo, after it)", "right",
         (body - 0.08 * r0[:, 2], r0, FINGER_OPEN), True),
        ("left fingertips 6 mm off the body, 7 cm up", "left", (BASE + (0.0, 0.064, 0.07), SIDE, FINGER_CLOSED), False),
        ("left fingertips 2 cm off the body, 7 cm up", "left", (BASE + (0.0, 0.08, 0.07), SIDE, FINGER_CLOSED), True),
    ]


def case_arms(side: str, args) -> dict[str, tuple[dict[str, float], np.ndarray]]:
    """arm key -> (joint angles, base position) for one case."""
    other = "left" if side == "right" else "right"
    moved, base, _ = arm_joints(side, *args)
    rest, rest_base, _ = arm_joints(other)
    return {f"{side}_robot": (moved, base), f"{other}_robot": (rest, rest_base)}


if __name__ == "__main__":
    T.arm_samples()
    print(f"let go: every part of both arms at least {T.LET_GO * 100:.0f} cm from the bottle")
    bad = 0
    for label, side, args, want in cases():
        poses = {arm: T.arm_link_poses(j, b) for arm, (j, b) in case_arms(side, args).items()}
        gap, nearest = T.arms_clearance(poses, BOTTLE)
        ok = (gap >= T.LET_GO) == want
        bad += not ok
        print(f"- {label}: nearest {nearest}, gap {gap * 1000:+.1f} mm -> let go: {gap >= T.LET_GO}"
              f"{'' if ok else '  UNEXPECTED'}")
    sys.exit(1 if bad else 0)

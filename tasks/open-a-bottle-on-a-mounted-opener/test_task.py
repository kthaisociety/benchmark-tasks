"""Offline tests of the scorer's runtime path (yam_arena and NumPy; no simulator): ``pytest test_task.py`` or
``python test_task.py``.

The simulator is replaced by a stand-in with Isaac Lab's data layout: per arm ``body_names``, ``joint_names`` and
``data.body_link_pose_w`` (1, bodies, 7), ``data.joint_pos`` (1, joints), ``data.body_pos_w``; per part
``data.root_link_pose_w`` (1, 7) and ``data.root_vel_w`` (1, 6); ``scene.env_origins``. The arms are posed with the
URDF's kinematics for check_release.py's cases (the bottle held by its neck, held by its body, touched on its body,
let go), and ``_follow`` then ``released()`` and ``score()`` run on that data exactly as they do in a trial.

Isaac Lab 3.0, which yam_arena runs on, orders pose quaternions (x, y, z, w); 2.x ordered them (w, x, y, z). The link
poses must not depend on that order: the stand-in is run with the arms' link quaternions in both orders.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import partkit  # noqa: E402
import task as T  # noqa: E402
from check_release import BASE, BOTTLE, case_arms, cases  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import LINKS  # noqa: E402
from yam_arena.urdf import forward_kinematics  # noqa: E402

from yam_arena import YAM_URDF  # noqa: E402,I001


class _Tensor:
    """Just enough of a torch tensor: indexing, subtraction and .detach().cpu().numpy()."""

    def __init__(self, a):
        self.a = np.asarray(a, dtype=np.float64)

    def __getitem__(self, i):
        return _Tensor(self.a[i])

    def __sub__(self, other):
        return _Tensor(self.a - (other.a if isinstance(other, _Tensor) else other))

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a.copy()


class _Data:
    def __init__(self, **buffers):
        for k, v in buffers.items():
            setattr(self, k, _Tensor(v))


class _Asset:
    def __init__(self, data, body_names=(), joint_names=()):
        self.data, self.body_names, self.joint_names = data, list(body_names), list(joint_names)


class _Scene(dict):
    env_origins = _Tensor([[0.0, 0.0, 0.0]])


class _Env:
    def __init__(self, scene):
        self.scene, self.unwrapped = scene, self


def _quat(R, order: str) -> np.ndarray:
    x, y, z, w = ik.matrix_to_quat_xyzw(R)
    return np.array([x, y, z, w] if order == "xyzw" else [w, x, y, z])


def _arm(joints: dict[str, float], base, order: str) -> _Asset:
    fk = forward_kinematics(YAM_URDF, joints)
    poses = np.array([[*(fk[n][:3, 3] + base), *_quat(fk[n][:3, :3], order)] for n in LINKS])
    names = [f"joint{i}" for i in range(1, 9)]
    return _Asset(_Data(body_link_pose_w=poses[None], body_pos_w=poses[None, :, :3],
                        joint_pos=np.array([[joints[n] for n in names]])), LINKS, names)


def _part(pos, quat_xyzw) -> _Asset:
    return _Asset(_Data(root_link_pose_w=np.array([[*pos, *quat_xyzw]]), root_vel_w=np.zeros((1, 6))))


def _env(arms: dict, link_order: str = "xyzw", bottle_quat=(0.0, 0.0, 0.0, 1.0), cap_quat=None) -> _Env:
    """The open bottle standing on the coaster, its cap 5 cm to the side on the table, and the arms."""
    scene = _Scene({key: _arm(j, b, link_order) for key, (j, b) in arms.items()})
    scene["bottle"] = _part(BASE, bottle_quat)
    scene["cap"] = _part(BASE + (0.06, 0.0, 0.0), cap_quat or bottle_quat)
    return _Env(scene)


def _fresh_trial(**state) -> None:
    partkit._SPECS.update({T.BOTTLE.name: T.BOTTLE, T.CAP.name: T.CAP})
    T._S.clear()
    T._S.update(T._fresh())
    T._S.update(state)


def _truth(arms) -> float:
    """The gap straight from the kinematics (rotation matrices; nothing read back from simulator data)."""
    return T.arms_clearance({k: T.arm_link_poses(j, b) for k, (j, b) in arms.items()}, BOTTLE)[0]


def _run_case(arms, link_order: str) -> tuple[float, bool, float]:
    _fresh_trial(engaged=True, released=True, popped=True, release_step=1, last_caught=1)
    env = _env(arms, link_order)
    T._follow(env)
    bottle = partkit._part_state(env, T.BOTTLE)
    return T._S["gap"], T.released(), T.score({T.BOTTLE.name: bottle})


def test_live_scorer_neck_body_touch_and_released():
    """Held by the neck, held by the body, touched on the body: 0.75; let go: 1.0. The gap the runtime path measures
    equals the kinematics' own, whichever order the simulator's link quaternions come in."""
    T.arm_samples()
    for label, side, args, let_go in cases():
        arms = case_arms(side, args)
        truth = _truth(arms)
        for order in ("xyzw", "wxyz"):
            gap, rel, s = _run_case(arms, order)
            assert abs(gap - truth) < 1e-9, f"{label} ({order}): runtime gap {gap * 1000:.1f} mm, " \
                                            f"kinematics {truth * 1000:.1f} mm"
            assert rel == let_go, f"{label} ({order}): released() {rel}, want {let_go}"
            assert s == (1.0 if let_go else 0.75), f"{label} ({order}): score {s}"


def test_link_positions_checked_against_the_simulator():
    """A link the kinematics put more than FK_TOL from the simulator's own link position (a base turned, a joint
    mislabelled) stops the let-go from passing at all."""
    label, side, args, _ = cases()[-1]  # let go
    arms = case_arms(side, args)
    env = _env(arms)
    env.scene["right_robot"].data.body_link_pose_w.a[0, LINKS.index("gripper"), 0] += 0.02
    _fresh_trial(engaged=True, released=True, popped=True, release_step=1, last_caught=1)
    T._follow(env)
    assert T._S["fk_error"] > T.FK_TOL and not T.released()


def test_reset_rejects_a_quaternion_order_it_does_not_read(monkeypatch):
    """The standing bottle read in the wrong order shows upside down: reset refuses to run the trial."""
    monkeypatch.setattr(T, "_seat_cap", lambda environment, bottle: None)
    arms = case_arms(*cases()[0][1:3])
    _fresh_trial()
    T._reset(_env(arms))  # Isaac Lab 3.0's order: fine
    try:
        T._reset(_env(arms, bottle_quat=(1.0, 0.0, 0.0, 0.0)))  # an upright bottle in (w, x, y, z)
    except RuntimeError as err:
        assert "upside down" in str(err)
    else:
        raise AssertionError("reset accepted a bottle reading upside down")


def _cap_on_lip(turn_deg: float = 0.0, crimp_deg: float = 0.0, hooked: bool = True):
    """Poses (pos, quat_xyzw) of the bottle and the cap in the Starr X's mouth: the cap's top on the pad, moved over so
    the lip is under the skirt's edge (or not, ``hooked=False``), the bottle then pressed ``turn_deg`` toward upright
    about the pad, the cap lagging ``crimp_deg`` behind it on the finish (held by the lip)."""
    tops = T._mouth_tops()
    base0 = T._base_for_top(tops["hooked" if hooked else "in"], T.TILT_IN)
    pivot = T.jaw_to_world(T.PIVOT_J)
    turn = -math.radians(turn_deg)
    b = T._tipped_about(base0, pivot, turn)
    Rb = T._ry(T.TILT_IN + turn)
    seat = b + Rb @ (0.0, 0.0, T.SEAT)
    Rc = T._ry(T.TILT_IN + turn + math.radians(crimp_deg))
    return (b, ik.matrix_to_quat_xyzw(Rb)), (seat, ik.matrix_to_quat_xyzw(Rc))


def _cap_state(cap):
    return partkit.PartState(pos=np.asarray(cap[0]), quat_xyzw=np.asarray(cap[1]), lin_vel=np.zeros(3),
                             ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))


def test_caught_follows_the_cap_as_it_tips():
    for turn in (0.0, 3.5, 6.0):
        assert T.caught(_cap_state(_cap_on_lip(turn)[1])), turn
    assert not T.caught(_cap_state(_cap_on_lip(0.0, hooked=False)[1]))  # no_hook: the lip never under the skirt


def _crimp_step(monkeypatch, bottle, cap, **state):
    switched = []
    monkeypatch.setattr(T, "set_joint_enabled", lambda name, on: switched.append((name, on)))
    arms = case_arms(*cases()[-1][1:3])
    env = _env(arms)
    env.scene["bottle"] = _part(*bottle)
    env.scene["cap"] = _part(*cap)
    _fresh_trial(**state)
    T._follow(env)
    return switched, T.score({T.BOTTLE.name: partkit._part_state(env, T.BOTTLE)})


def test_crimp_holds_below_its_opening_moment(monkeypatch):
    bottle, cap = _cap_on_lip(3.5 + math.degrees(T.THETA_POP) - 0.5, crimp_deg=math.degrees(T.THETA_POP) - 0.5)
    switched, s = _crimp_step(monkeypatch, bottle, cap)
    assert not switched and not T._S["released"] and T._S["engaged"] and s == 0.25
    assert 0.8 < T._S["moment"] < T.T_OPEN


def test_crimp_lets_go_at_its_opening_moment_on_the_lip(monkeypatch):
    bottle, cap = _cap_on_lip(3.5 + math.degrees(T.THETA_POP) + 0.1, crimp_deg=math.degrees(T.THETA_POP) + 0.1)
    switched, s = _crimp_step(monkeypatch, bottle, cap)
    assert switched == [(T.CRIMP.name, False)] and T._S["popped"] and T._S["moment"] >= T.T_OPEN and s == 0.5


def test_crimp_pried_open_off_the_lip_scores_nothing(monkeypatch):
    bottle, cap = _cap_on_lip(6.0, crimp_deg=math.degrees(T.THETA_POP) + 0.1, hooked=False)
    switched, s = _crimp_step(monkeypatch, bottle, cap)
    assert switched and T._S["released"] and not T._S["popped"] and s == 0.0


def test_crimp_numbers():
    """0.9 N m (8 lbf in) at a 1.5 mm lift of the skirt's edge: 6 degrees, 8.6 N m/rad."""
    assert abs(T.T_OPEN / 0.113 - 8.0) < 0.1
    assert abs(math.degrees(T.THETA_POP) - 5.97) < 0.01 and abs(T.CRIMP_K - 8.64) < 0.01


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-q"]))


def test_starr_x_jaw_meets_the_cap_where_the_patent_puts_it():
    """With the cap's top on the pad and the lip under its skirt: the pad's nose is over the middle of the cap's top,
    the lip is 0.9 mm in under the skirt's 1 mm wall and 1 mm beyond its edge, and the opener's back is on the post."""
    nose = np.array(T.OPENER.boxes[0][0]) + np.asarray(T.PAD_AT)
    _, cap = _cap_on_lip(0.0)
    c = _cap_state(cap)
    top = c.to_local(nose)
    assert abs(top[0]) < 0.0006 and abs(top[1]) < 1e-9  # over the middle (the hook leaves it 0.5 mm off)
    assert T.caught(c)
    assert abs(T.WALL_X - T.PAD_AT[0] - 0.015) < 1e-12

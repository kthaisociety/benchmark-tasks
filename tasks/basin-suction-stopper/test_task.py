"""Offline checks for the basin suction-stopper task: scene geometry, the seal model, the scoring rule, the demo plans'
reach, the arms' clearance from the basin, faucet and soap dish along every demo path, and the pull the arm can make.
No Isaac Sim needed (PYTHONPATH must hold yam_arena)."""
import itertools
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ.setdefault("SINK_DEBUG", "0")
import task  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

G = 9.81
VARIANTS = ("full", "weak_pull", "drop_back", "counter", "keep_holding")


# --- helpers -------------------------------------------------------------------------------------------------------
def obstacles():
    """(name, centre, rotation, size) boxes for everything fixed the arms must not hit: the basin's wall panels and
    floor, the faucet and the soap dish (the stopper is not an obstacle; the counter is checked in clearance())."""
    out = []
    for part in (task.BASIN, task.FAUCET, task.DISH):
        for i, spec in enumerate(part.boxes):
            centre, size, _, rot, visual = task.partkit._box(spec)
            if visual:
                continue
            out.append((f"{part.name}{i}", np.array(part.pos) + np.array(centre), task.partkit._quat_matrix(rot),
                        np.array(size)))
    side = 2 * task.R_IN / math.sqrt(2)                                     # a square inside the round floor
    out.append(("basin_floor", np.array([task.XC, task.YC, task.TOP + task.FLOOR_T / 2]), np.eye(3),
                np.array([side, side, task.FLOOR_T])))
    return out


def _surf_points(c, rot, size, step=0.006):
    hs = size / 2
    axes = [np.linspace(-h, h, max(2, int(2 * h / step) + 1)) for h in hs]
    pts = []
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        for s in (-1, 1):
            for u, v in itertools.product(axes[j], axes[k]):
                p = np.zeros(3)
                p[i], p[j], p[k] = s * hs[i], u, v
                pts.append(p)
    return np.array(pts) @ rot.T + c


_OBST = None


def clearance(hulls):
    """(min signed distance between the arm's hulls and the obstacles, link, obstacle); negative = overlap. The
    counter is a plane: a convex hull's lowest point is one of its vertices."""
    global _OBST
    if _OBST is None:
        _OBST = [(n, c, r, s, _surf_points(c, r, s)) for n, c, r, s in obstacles()]
        _OBST = [(*o, o[4].min(0) - 0.05, o[4].max(0) + 0.05) for o in _OBST]
    low = min((lh for lh in hulls if lh[0] != "base"), key=lambda lh: lh[1].vertices[:, 2].min())   # base: on its mount
    best = (float(low[1].vertices[:, 2].min()) - task.TOP, low[0], "counter")
    boxes = [(ln, h, h.vertices.min(0), h.vertices.max(0)) for ln, h in hulls]
    for name, c, rot, size, pts, olo, ohi in _OBST:
        for ln, h, hlo, hhi in boxes:
            if np.any(hlo > ohi) or np.any(hhi < olo):
                continue
            lo, hi = hlo - 0.05, hhi + 0.05
            m = np.all((pts > lo) & (pts < hi), axis=1)
            if not m.any():
                continue
            d = min(float(analysis._signed(pts[m], h).min()),
                    float(analysis.box_signed_distance(h.vertices, c, rot, size).min()))
            if d < best[0]:
                best = (d, ln, name)
    return best


def arm_hulls(arm, q, finger):
    qd = {n: float(v) for n, v in zip(ARM_JOINTS, q)}
    qd["joint7"] = qd["joint8"] = float(finger)
    return analysis.world_hulls(analysis.fk_link_poses_world(qd, np.array(station.BASES[arm])))


def walk(variant, dt=0.25, arms=("left", "right")):
    """Yield (arm, t, label, q, finger, plan) along a variant's demo plan, as the scripted policy would solve it."""
    timed, _, _ = task.plans(variant)
    for arm, steps in timed.items():
        if arm not in arms:
            continue
        plan = ArmPlan(task._plan(steps), station.BASES[arm])
        for t in np.arange(0.0, plan.duration + 1e-9, dt):
            _, label = plan.action7(float(t))
            yield arm, float(t), label, plan.q.copy(), plan.target(float(t))[2], plan


def tcp_stiffness(arm, q):
    """3x3 translational stiffness at the TCP (N/m) from the joint stiffnesses, wrist free to turn (a soft bound)."""
    base = np.array(station.BASES[arm])
    p0, _ = ik.tcp_pose(q, base)
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += 1e-6
        jac[:, i] = (ik.tcp_pose(dq, base)[0] - p0) / 1e-6
    kq = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    return np.linalg.inv(jac @ np.linalg.inv(kq) @ jac.T), jac


# --- scene ---------------------------------------------------------------------------------------------------------
def test_scene_geometry():
    # full-size parts: a 16 in x 5.5 in vessel basin, a 1 3/4 in drain, a 7.5 cm stopper, a 17 cm-reach faucet
    assert abs(2 * task.R_OUT - 0.406) < 0.002 and abs(task.H_BASIN - 0.14) < 1e-9
    assert abs(2 * task.DRAIN_R - 0.044) < 0.001
    assert task.DISC_R - task.DRAIN_R >= 0.015                       # the disc laps the drain by 15 mm or more
    # the knob is wide enough to hold (grips under ~20 mm slip) and fits the open gripper (94 mm)
    assert 0.02 <= 2 * task.KNOB_R < 2 * abs(FINGER_OPEN) - 0.04
    # the stopper starts centred on the drain, flat on the floor: sealed and within the break limits
    lift, off = task.seal_findings(np.array(task.STOPPER.pos), np.eye(3))
    assert abs(lift) < 1e-9 and off < 1e-9 and not task.seal_breaks(lift, off)
    # the spout's outlet hangs over the basin, behind the knob and above the rim, within the stopper's reach of it
    assert task.XC < task.OUTLET_X < task.XC + task.R_IN - 0.05
    assert task.OUTLET_X - task.SPOUT_W / 2 - task.XC < task.DISC_R  # the spout's end is over the stopper's back edge
    assert task.OUTLET_Z > task.H_BASIN + 0.05                       # 8 cm over the rim
    # the faucet stands on the counter behind the basin, clear of it
    assert task.FAUCET_X - task.PLATE_W / 2 > task.XC + task.R_OUT + 0.005
    # the soap dish holds the stopper with room to spare and stands clear of the basin
    assert min(task.DISH_IN) > 2 * task.DISC_R + 0.02
    dx, dy = task.DISH_XY
    corners = [(dx + sx * task.DISH_OUT[0] / 2, dy + sy * task.DISH_OUT[1] / 2) for sx in (-1, 1) for sy in (-1, 1)]
    assert min(math.hypot(x - task.XC, y - task.YC) for x, y in corners) > task.R_OUT + 0.03
    # the 'counter' variant's spot is clear of the dish and the basin
    cx, cy = task.COUNTER_XY
    assert cx - task.DISC_R > dx + task.DISH_OUT[0] / 2 + 0.02
    assert math.hypot(cx - task.XC, cy - task.YC) > task.R_OUT + task.DISC_R + 0.02


def test_object_names_and_load():
    from yam_arena import task_file
    t = task_file.load(Path(task.__file__))
    assert t.names == ("basin", "faucet", "soap_dish", "stopper") and t.free_objects == ("stopper",)
    assert t.success_hold_s == 2.0 and t.score_over == "end"


# --- seal ----------------------------------------------------------------------------------------------------------
def _tilt(deg, axis=(0.0, 1.0, 0.0)):
    return axis_angle_matrix(np.array(axis), math.radians(deg))


def test_seal_breaks_on_lift_tilt_or_slide():
    p0 = np.array(task.STOPPER.pos, float)
    # lifted straight up: holds to 3 mm, lets go past it
    assert not task.seal_breaks(*task.seal_findings(p0 + [0, 0, 0.0029], np.eye(3)))
    assert task.seal_breaks(*task.seal_findings(p0 + [0, 0, 0.0031], np.eye(3)))
    # tilted about the near rim point (the far rim lifts 2 R sin th): 2.3 deg lifts it 3 mm
    for deg, broken in ((2.0, False), (2.5, True)):
        r = _tilt(-deg)                                             # -deg about y raises +x
        pivot = p0 + np.array([-task.DISC_R, 0, 0])
        pos = pivot + r @ np.array([task.DISC_R, 0, 0])
        assert task.seal_breaks(*task.seal_findings(pos, r)) is broken
    # slid sideways: lets go once the rim reaches the drain opening (15.5 mm)
    assert not task.seal_breaks(*task.seal_findings(p0 + [0.015, 0, 0], np.eye(3)))
    assert task.seal_breaks(*task.seal_findings(p0 + [0.0, -0.016, 0], np.eye(3)))


def test_seal_forces_are_consistent():
    w = task.STOPPER_MASS * G
    straight = task.F_SEAL + w                                     # straight pull to let go
    peel = (task.F_SEAL + w) * task.DISC_R / (task.STOPPER_H - 0.002)   # sideways push at the knob to tip it
    slide = 0.75 * (task.F_SEAL + w)                                # sideways push to slide it (mu 0.5/1.0 averaged)
    assert 9.5 < straight < 11.5
    assert slide < straight < peel                                  # sliding is the easiest, tipping the hardest
    # the grip can hold the pull: 50 N per finger, rubber on the pads
    assert 2 * 50.0 * 0.8 > 2 * straight


# --- scoring -------------------------------------------------------------------------------------------------------
def test_rule():
    r = task.rule
    assert r(True, True, False, False, True, True) == 0.0           # still sealed
    assert r(True, False, True, False, True, True) == 0.0           # sealed wins over anything else
    assert r(False, True, False, False, True, True) == 0.25         # unsealed, still in the basin
    assert r(False, False, False, True, True, True) == 0.25         # dropped off the counter
    assert r(False, False, False, False, True, True) == 0.5         # on the counter
    assert r(False, False, False, False, False, False) == 0.5       # held in the air
    assert r(False, False, True, False, False, True) == 0.75        # in the dish, still held
    assert r(False, False, True, False, True, False) == 0.75        # in the dish, still moving
    assert r(False, False, True, False, True, True) == 1.0


def test_where():
    mid = lambda x, y, z: np.array([x, y, task.TOP + z])                     # noqa: E731
    assert task.where(mid(task.XC, task.YC, 0.029)) == (True, False, False)  # on the drain
    assert task.where(mid(task.XC + 0.15, 0.05, 0.03)) == (True, False, False)
    assert task.where(mid(task.XC, task.YC, 0.30)) == (False, False, False)  # held high over the basin
    dx, dy = task.DISH_XY
    assert task.where(mid(dx, dy, 0.019)) == (False, True, False)            # lying in the dish
    assert task.where(mid(dx + 0.03, dy - 0.02, 0.025)) == (False, True, False)
    assert task.where(mid(dx, dy, 0.08)) == (False, False, False)            # held over the dish
    assert task.where(mid(*task.COUNTER_XY, 0.014)) == (False, False, False)
    assert task.where(mid(0.1, 0.5, -0.5)) == (False, False, True)           # on the floor


# --- demo ----------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_reach(variant):
    worst = 0.0
    for _, _, _, _, _, plan in walk(variant, dt=0.25):
        worst = max(worst, plan.worst_ik_mm)
    assert worst < 3.0, f"{variant}: IK error {worst:.1f} mm"


def test_straight_probe_reaches_the_knob():
    """The straight-down probe turns its wrist on the way in (a brief IK lag there), but reaches the knob itself."""
    timed, _, keys = task.plans("straight")
    plan = ArmPlan(task._plan(timed["right"]), station.BASES["right"])
    for t in np.arange(0.0, 6.5, 0.25):
        plan.action7(float(t))
    p, _ = ik.tcp_pose(plan.q, np.array(station.BASES["right"]))
    assert np.linalg.norm(p - keys["G"]) < 0.002


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_clears_the_basin_faucet_and_dish(variant):
    worst = (np.inf, None)
    timed, _, _ = task.plans(variant)
    assert all(np.allclose(p, timed["left"][0][1]) for _, p, _, _, _ in timed["left"])   # the left arm only waits
    for arm, t, label, q, finger, _ in itertools.chain(walk(variant, dt=100.0, arms=("left",)),
                                                       walk(variant, arms=("right",))):
        d, link, obst = clearance(arm_hulls(arm, q, finger))
        if d < worst[0]:
            worst = (d, f"{arm} {link}/{obst} at {t:.2f} s ({label})")
    assert worst[0] > 0.01, f"{variant}: {worst[0] * 100:.1f} cm, {worst[1]}"


def test_spout_hinders_a_straight_down_grasp():
    """With a plain downward grip the hand comes down within a centimetre of the spout (or into it); leaned back it
    clears by several centimetres. This is the faucet's hindrance, measured on the hulls."""
    clearance(arm_hulls("right", np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), FINGER_OPEN))   # builds the cache
    spout = [o for o in _OBST if o[0].startswith("faucet")]

    def min_to_spout(variant):
        best = np.inf
        for arm, t, label, q, finger, _ in walk(variant, arms=("right",)):
            if t > 7.0:
                break
            for ln, h in arm_hulls(arm, q, finger):
                for name, c, rot, size, pts, _, _ in spout:
                    best = min(best, float(analysis._signed(pts, h).min()),
                               float(analysis.box_signed_distance(h.vertices, c, rot, size).min()))
        return best
    assert min_to_spout("straight") < 0.01
    assert min_to_spout("full") > 0.03


def test_pull_force_budget():
    """The arm is a spring (joint stiffness 80/10 N m/rad): drawing the TCP back PULL along the leaned axis with the
    knob held fast pulls well over the seal's 10.4 N; the weak_pull variant's 1 cm stays under it. Joint torques for
    that pull stay inside the motors' limits."""
    timed, _, keys = task.plans("full")
    plan = ArmPlan(task._plan(timed["right"]), station.BASES["right"])
    for t in np.arange(0.0, 6.5, 0.25):
        plan.action7(float(t))
    q = plan.q.copy()
    k, jac = tcp_stiffness("right", q)
    a = keys["rot"][:, 2]
    seal = task.F_SEAL + task.STOPPER_MASS * G
    f_full = k @ (-a * task.PULL)
    f_weak = k @ (-a * 0.01)
    assert f_full[2] > 1.3 * seal, f"full pull lifts with {f_full[2]:.1f} N"
    assert np.linalg.norm(f_weak) < 0.8 * seal, f"weak pull {np.linalg.norm(f_weak):.1f} N"
    tau = np.abs(jac.T @ f_full)
    limits = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
    assert np.all(tau < 0.5 * limits), tau

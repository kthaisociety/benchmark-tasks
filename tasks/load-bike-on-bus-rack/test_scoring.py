"""Offline checks of the scorer and the rack geometry, no simulator: python -m pytest -q test_scoring.py"""

import math
from types import SimpleNamespace

import numpy as np

import bike as B
import layout as L
import rack as K
import scoring as S

PARK_L, PARK_R = (0.30, 0.27, L.TOP + 0.20), (0.30, -0.27, L.TOP + 0.20)


def quat(yaw=0.0, lean_deg=0.0):
    """xyzw: lean about the bike's own long (y) axis, then yaw about vertical."""
    a, b = yaw / 2, math.radians(lean_deg) / 2
    qz = np.array((0.0, 0.0, math.sin(a), math.cos(a)))
    qy = np.array((0.0, math.sin(b), 0.0, math.cos(b)))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    return np.array((w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2))


def bike(x, y, z=0.0, yaw=0.0, lean_deg=0.0, moving=False):
    v = np.array((0.05, 0.0, 0.0)) if moving else np.zeros(3)
    return SimpleNamespace(pos=np.array((x, y, L.TOP + z)), quat_xyzw=quat(yaw, lean_deg), lin_vel=v, ang_vel=np.zeros(3))


def rack(outer=0.0, inner=0.0, shift=(0.0, 0.0, 0.0)):
    p = np.array(L.RACK_ORIGIN) + np.array((0.0, 0.0, L.TOP)) + np.asarray(shift)
    return SimpleNamespace(pos=p, quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                           sliders={"outer": outer, "inner": inner})


def tcp(p):
    return SimpleNamespace(pos=np.asarray(p, float))


def scene(b, r, left=PARK_L, right=PARK_R):
    return {"bike": b, "rack": r, "left_tcp": tcp(left), "right_tcp": tcp(right)}


def run(objects, steps=1):
    t = S.Trial()
    out = [t.step(objects) for _ in range(steps)]
    return out[-1], t.fault


def landing(b, name, target):
    """The hinge angle at which `target(objects)` first holds, swinging the arm out from stowed in 0.05 degree steps."""
    for q in np.arange(0.0, K.q_low(name), -math.radians(0.05)):
        o = scene(b, rack(**{name: q}))
        if target(o):
            return q
    raise AssertionError("never reached")


def touches_front(name):
    """The hook pad's bottom has come down to the front tyre's surface (within 2 mm)."""
    def f(o):
        v = S.hook_world(o, name) - S.wheel(o, "front")
        return abs(v[0]) < 0.02 and np.linalg.norm(v[1:]) <= B.R + 0.002
    return f


def touches_rear(name):
    def f(o):
        v = S.hook_world(o, name) - S.wheel(o, "rear")
        return abs(v[0]) < 0.02 and np.linalg.norm(v[1:]) <= B.R + 0.002
    return f


OX, IX = L.TRAYS["outer"], L.TRAYS["inner"]
Y0 = L.BIKE0[1]
GRIP = (OX, Y0 + B.GRIP_Y, L.TOP + B.TOP_TUBE["z"] + 0.013)


def test_rest_geometry_puts_the_hook_on_a_centred_bikes_front_tyre():
    o = scene(bike(OX, Y0), rack(outer=K.q_rest("outer")))
    assert S.hook_on_front_tyre(o, "outer") == ""
    o = scene(bike(IX, Y0, yaw=math.pi), rack(inner=K.q_rest("inner")))
    assert S.hook_on_front_tyre(o, "inner") == ""
    assert K.q_low("outer") < K.q_rest("outer") < 0 and K.q_low("inner") < K.q_rest("inner") < 0


def test_start_is_zero_and_clean():
    assert run(scene(bike(*L.BIKE0), rack())) == (0.0, "")


def test_in_outer_tray_held_without_the_arm_is_a_quarter():
    assert run(scene(bike(OX, Y0), rack(), right=GRIP)) == (0.25, "")


def test_secured_but_still_held_is_point_six():
    assert run(scene(bike(OX, Y0), rack(outer=K.q_rest("outer")), right=GRIP), steps=40) == (0.6, "")


def test_secured_let_go_and_still_for_a_second_is_full():
    o = scene(bike(OX, Y0), rack(outer=K.q_rest("outer")))
    assert run(o, steps=19)[0] == 0.6
    assert run(o, steps=20) == (1.0, "")
    moving = scene(bike(OX, Y0, moving=True), rack(outer=K.q_rest("outer")))
    assert run(moving, steps=40)[0] == 0.6


def test_hand_on_the_hook_is_not_let_go():
    q = K.q_rest("outer")
    hand = K.hook("outer", q) + np.array((0.0, 0.0, L.TOP + 0.02))
    assert run(scene(bike(OX, Y0), rack(outer=q), left=hand), steps=40)[0] == 0.6


def test_physical_landing_spot_scores_like_the_design():
    b = bike(OX, Y0)
    q = landing(b, "outer", touches_front("outer"))
    assert run(scene(b, rack(outer=q)), steps=20) == (1.0, "")


def test_hook_on_the_brake_is_not_secured():
    # the bike slid 2.2 cm toward the hook end: the pad comes down on the fork crown and brake caliper
    b = bike(OX, Y0 + L.FRONT_OFFSET)
    crown_top = L.TOP + B.CROWN["z1"]
    q = landing(b, "outer", lambda o: S.hook_world(o, "outer")[2] <= crown_top)
    o = scene(b, rack(outer=q))
    assert S.hook_on_front_tyre(o, "outer") == "not touching the front tyre"
    assert run(o, steps=40) == (0.25, "")


def test_hook_far_from_the_fork_is_not_secured():
    b = bike(OX, Y0 - 0.025)
    q = landing(b, "outer", touches_front("outer"))
    o = scene(b, rack(outer=q))
    assert S.hook_on_front_tyre(o, "outer") == "too far from the fork"
    assert run(o, steps=40) == (0.25, "")


def test_reversed_bike_with_the_arm_on_the_rear_wheel_is_a_quarter():
    b = bike(OX, Y0, yaw=math.pi)
    q = landing(b, "outer", touches_rear("outer"))
    o = scene(b, rack(outer=q))
    assert not S.faces_hook(o, "outer")
    assert run(o, steps=40) == (0.25, "")


def test_inner_position_secured_is_half():
    b = bike(IX, Y0, yaw=math.pi)  # front wheel toward the inner hook's end (-y)
    q = landing(b, "inner", touches_front("inner"))
    assert run(scene(b, rack(inner=q)), steps=40) == (0.5, "")
    assert run(scene(b, rack(inner=q), right=(IX, Y0 - B.GRIP_Y, GRIP[2])), steps=40) == (0.25, "")


def test_inner_position_facing_the_start_way_is_a_quarter():
    b = bike(IX, Y0)  # front wheel at +y; the inner hook is at -y, so its arm lands on the rear wheel
    q = landing(b, "inner", touches_rear("inner"))
    assert run(scene(b, rack(inner=q)), steps=40) == (0.25, "")


def test_outer_arm_over_a_bike_in_the_inner_tray_does_not_count():
    b = bike(IX, Y0)
    q = landing(b, "outer", lambda o: S.hook_world(o, "outer")[2] <= L.TOP + 2 * B.R + 0.01)
    assert run(scene(b, rack(outer=q)), steps=40) == (0.25, "")


def test_falling_over_is_latched():
    t = S.Trial()
    t.step(scene(bike(OX, Y0, lean_deg=30.0), rack()))
    assert t.fault.startswith("bike fell over")
    assert max(t.step(scene(bike(OX, Y0), rack(outer=K.q_rest("outer")))) for _ in range(60)) == 0.0


def test_lean_limits():
    q = K.q_rest("outer")
    assert run(scene(bike(OX, Y0, lean_deg=5.0), rack(outer=q)), steps=20) == (1.0, "")
    assert run(scene(bike(OX, Y0, lean_deg=15.0), rack(outer=q)), steps=20) == (0.25, "")


def test_moving_the_rack_is_latched():
    assert run(scene(bike(*L.BIKE0), rack(shift=(0.02, 0.0, 0.0))))[1] == "rack moved"


def test_bike_held_above_the_tray_or_left_in_the_rail_is_zero():
    assert run(scene(bike(OX, Y0, z=0.03), rack(), right=GRIP)) == (0.0, "")
    assert run(scene(bike(*L.BIKE0), rack(outer=K.q_low("outer")))) == (0.0, "")


def test_every_wheel_outside_any_tray_is_zero():
    assert run(scene(bike((OX + IX) / 2, Y0), rack())) == (0.0, "")


def _first_contact(dy):
    """Swing the outer arm out from stowed over a bike in the outer channels shifted dy along them; return the hinge
    angle and the bike boxes its boxes first overlap (points sampled inside each arm box)."""
    shapes = B.boxes()
    off = np.array((OX, Y0 + dy, 0.0))
    grid = np.stack(np.meshgrid(*[np.linspace(-0.5, 0.5, 7)] * 3), -1).reshape(-1, 3)
    for q in np.arange(0.0, K.q_low("outer"), -math.radians(0.5)):
        pts = np.vstack([np.array([K.point_at("outer", c + K.ry(rot) @ (g * np.array(s)), q) for g in grid])
                         for c, s, _, rot in K.arm_shapes("outer")])
        hit = [i for i, (c, s, _, w) in enumerate(shapes)
               if np.any(np.all(np.abs((pts - (np.array(c) + off)) @ B._rot(w)) < np.array(s) / 2 - 0.0005, axis=1))]
        if hit:
            return q, hit
    return None, []


def test_the_swing_clears_the_bike_until_it_meets_the_front_tyre():
    tyre = set(range(B.SEGMENTS, 2 * B.SEGMENTS))  # the front wheel's tyre segments (boxes() lists rear wheel first)
    q, hit = _first_contact(0.0)
    assert set(hit) <= tyre and abs(q - K.q_rest("outer")) < math.radians(2)
    q, hit = _first_contact(L.FRONT_OFFSET)  # slid toward the hook end: meets the frame/brake, not the tyre
    assert hit and not set(hit) & tyre

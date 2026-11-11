"""Offline tests of the cuff scoring, geometry and demo plan: no Isaac Sim, yam_arena, network or GPU needed.

    python3 -m unittest discover -v -s tasks/bp-cuff-size -p test_task.py
"""

import itertools
import math
import unittest
from types import SimpleNamespace

import numpy as np

import cuff_scoring as cs

TOP = 0.75
RIGHT = "cuff_8"
WRONG = ("cuff_4", "cuff_12")


def quat(yaw_deg=0.0, tilt_deg=0.0):
    """xyzw quaternion: yaw about z, then tilt about x."""
    a, b = math.radians(tilt_deg) / 2, math.radians(yaw_deg) / 2
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    return (sa * cb, sa * sb, ca * sb, ca * cb)


def state(x, y, z=0.0, yaw=0.0, tilt=0.0):
    return SimpleNamespace(pos=np.array([x, y, TOP + z]), quat_xyzw=quat(yaw, tilt))


def start_scene(**placed):
    """Every cuff at its start (spawn height), except those given as name=state."""
    states = {n: state(cs.START_X, cs.START_Y[n], cs.SPAWN_LIFT) for n in cs.CUFF_NAMES}
    states.update(placed)
    return states


def on_arm(y, **kw):
    """A cuff centred at ``y`` along the arm, standing on the table astride it (``dx`` off the axis)."""
    return state(cs.LIMB_X + kw.pop("dx", 0.0), y, kw.pop("z", 0.0), **kw)


def score(states):
    return cs.evaluate(states, TOP)


def reach_ok(x, y):
    return any(cs.REACH_M[0] <= math.hypot(x - bx, y - by) <= cs.REACH_M[1] for bx, by in (cs.RIGHT_BASE,))


class Geometry(unittest.TestCase):
    def test_only_the_8_cm_cuff_is_the_right_size(self):
        self.assertEqual(cs.right_size(), [RIGHT])
        self.assertAlmostEqual(cs.CIRCUMFERENCE, 0.20)
        self.assertAlmostEqual(cs.ratio("cuff_8"), 0.40)
        self.assertLess(cs.ratio("cuff_4"), cs.RATIO_LO - 0.10)       # clear of the band, so a few mm never matters
        self.assertGreater(cs.ratio("cuff_12"), cs.RATIO_HI + 0.05)

    def test_every_cuff_fits_over_the_arm_with_room(self):
        self.assertGreaterEqual(cs.CLEAR, 0.015)                       # robot limit: at least 1.5 cm free each side
        self.assertGreater(cs.LEG_H, cs.LIMB_H + 0.005)                # the bridge clears the arm
        for name, w in cs.WIDTHS.items():
            # turned by the yaw tolerance, the cuff's legs still clear the arm's corners
            half = cs.LIMB_W / 2 * math.cos(math.radians(cs.YAW_TOL_DEG)) + w / 2 * math.sin(math.radians(cs.YAW_TOL_DEG))
            self.assertLessEqual(half, cs.INNER / 2, name)

    def test_every_cuff_fits_the_upper_arm_lengthwise_so_size_is_the_only_difference(self):
        room = cs.UPPER_Y[1] - (cs.UPPER_Y[0] + cs.ELBOW_GAP)
        for name, w in cs.WIDTHS.items():
            self.assertLessEqual(w + 0.02, room, name)                 # with 2 cm of play

    def test_grip_knob_is_graspable(self):
        self.assertGreaterEqual(cs.KNOB[2], 0.045 + 0.004)             # robot limit ~4.5 cm tall; keep a margin
        self.assertEqual(cs.KNOB[0], cs.KNOB[1])                       # square: either finger axis grips it
        self.assertLess(cs.KNOB[1], min(cs.WIDTHS.values()))           # sits inside the narrowest cuff
        self.assertLess(cs.GRIP_VALUE, 0.0)

    def test_the_row_is_in_reach_apart_and_middles_the_right_cuff(self):
        ys = sorted(cs.START_Y.values())
        self.assertEqual(cs.START_Y[RIGHT], ys[1])
        for a, b in itertools.combinations(cs.CUFF_NAMES, 2):
            self.assertGreaterEqual(abs(cs.START_Y[a] - cs.START_Y[b]), 0.12)               # placements 12 cm apart
            gap = abs(cs.START_Y[a] - cs.START_Y[b]) - (cs.WIDTHS[a] + cs.WIDTHS[b]) / 2
            self.assertGreaterEqual(gap, 0.025, (a, b))                                      # clear of each other
        for n in cs.CUFF_NAMES:
            self.assertTrue(reach_ok(cs.START_X, cs.START_Y[n]), n)
            self.assertLessEqual(cs.START_Y[n], 0.15)                  # an arm can't reach ~15 cm past the midline

    def test_the_row_is_clear_of_the_arm(self):
        gap = (cs.LIMB_X - cs.LIMB_W / 2) - (cs.START_X + cs.OUTER / 2)
        self.assertGreaterEqual(gap, 0.06)

    def test_nothing_starts_where_the_parked_arms_hide_it_from_the_top_camera(self):
        # lesson: keep things out of x < 0.30 with |y| > 0.20 (the arms' parking pose covers it)
        for name, x0, x1, y0, y1 in cs.footprints():
            self.assertFalse(x0 < 0.30 and max(abs(y0), abs(y1)) > 0.20, name)

    def test_the_full_score_spot_is_in_reach_of_both_arms(self):
        for y in (cs.ZONE_CENTRE_Y - 0.04, cs.ZONE_CENTRE_Y, cs.ZONE_CENTRE_Y + 0.04):
            for base in (cs.RIGHT_BASE, cs.LEFT_BASE):
                d = math.hypot(cs.LIMB_X - base[0], y - base[1])
                self.assertTrue(cs.REACH_M[0] <= d <= cs.REACH_M[1], (y, base, d))

    def test_the_start_is_not_over_the_arm_so_waiting_scores_zero(self):
        for n in cs.CUFF_NAMES:
            self.assertGreater(abs(cs.START_X - cs.LIMB_X), cs.OVER_LATERAL + 0.10)

    def test_masses_are_in_the_carry_limit(self):
        self.assertLessEqual(max(cs.cuff_mass(w) for w in cs.WIDTHS.values()), 0.25)

    def test_part_positions_are_plain_floats(self):
        # lesson: numpy float64 in part positions crashes Warp
        for v in (cs.START_X, *cs.START_Y.values(), cs.LIMB_X, *cs.LIMB_Y):
            self.assertIs(type(v), float)


class Scoring(unittest.TestCase):
    def test_start_scores_zero(self):
        self.assertEqual(score(start_scene()), 0.0)

    def test_right_cuff_on_the_upper_arm_is_full_marks(self):
        for y in (-0.04, 0.0, 0.04):                                   # the whole allowed range for the 8 cm cuff
            self.assertEqual(score(start_scene(cuff_8=on_arm(y))), 1.0, y)

    def test_zone_edges(self):
        lo, hi = cs.UPPER_Y[0] + cs.ELBOW_GAP + 0.04, cs.UPPER_Y[1] - 0.04
        self.assertEqual(score(start_scene(cuff_8=on_arm(lo + 0.0005))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(lo - 0.002))), 0.5)   # lower edge under 2 cm above the crease
        self.assertEqual(score(start_scene(cuff_8=on_arm(hi - 0.0005))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(hi + 0.002))), 0.25)  # overhangs the shoulder end: not on it

    def test_right_cuff_in_the_wrong_place_on_the_arm_is_half(self):
        self.assertEqual(score(start_scene(cuff_8=on_arm(-0.15))), 0.5)         # over the forearm
        self.assertEqual(score(start_scene(cuff_8=on_arm(-0.10))), 0.5)         # over the crease
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, yaw=20))), 0.5)   # turned past 12 degrees
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, yaw=10))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, yaw=190))), 1.0)  # a half turn is the same cuff

    def test_a_wrong_width_cuff_never_scores_on_its_own(self):
        for name in WRONG:
            for y in np.arange(-0.19, 0.08, 0.01):
                for dx in (-0.02, 0.0, 0.02):
                    for z in (0.0, 0.10):
                        self.assertEqual(score(start_scene(**{name: on_arm(float(y), dx=dx, z=z)})), 0.0, (name, y, dx))

    def test_only_the_right_cuff_can_reach_full_marks(self):
        for name in cs.CUFF_NAMES:
            best = max(score(start_scene(**{name: on_arm(float(y), dx=dx)}))
                       for y in np.arange(-0.22, 0.10, 0.005) for dx in (-0.02, -0.01, 0.0, 0.01, 0.02))
            self.assertEqual(best, 1.0 if name == RIGHT else 0.0, name)

    def test_an_extra_cuff_on_the_arm_costs_half(self):
        for name, y in (("cuff_4", -0.15), ("cuff_12", -0.13)):
            self.assertEqual(score(start_scene(cuff_8=on_arm(0.0), **{name: on_arm(y)})), 0.5, name)

    def test_an_extra_cuff_beside_the_arm_does_not(self):
        beside = state(cs.LIMB_X - 0.11, -0.13, cs.SPAWN_LIFT)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0), cuff_12=beside)), 1.0)

    def test_an_extra_cuff_perched_on_top_of_the_arm_does_not_count_as_on_it(self):
        perched = on_arm(-0.15, z=cs.LIMB_H + 0.001, tilt=20)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0), cuff_4=perched)), 1.0)

    def test_over_the_arm_but_not_standing_on_the_table_is_a_quarter(self):
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, z=0.10))), 0.25)                   # held over it
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.035, dx=0.039, z=0.089, tilt=13))), 0.25)  # as held in the build
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, z=cs.LIMB_H + 0.001, tilt=20))), 0.25)  # perched on it
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, dx=0.03))), 0.25)                 # off-centre, a leg on it

    def test_set_down_beside_the_arm_is_zero(self):
        self.assertEqual(score(start_scene(cuff_8=state(0.24, -0.03, cs.SPAWN_LIFT))), 0.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, dx=0.06))), 0.0)

    def test_not_upright_is_zero(self):
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, tilt=90))), 0.0)       # on its side across the arm
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, tilt=180))), 0.0)      # upside down
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, tilt=10))), 0.25)      # tilted past 8, still over it

    def test_lateral_and_seating_limits(self):
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, dx=0.024))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, dx=-0.024))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, z=0.005))), 1.0)
        self.assertEqual(score(start_scene(cuff_8=on_arm(0.0, z=0.02))), 0.25)

    def test_overhanging_an_end_of_the_arm_is_not_on_it(self):
        self.assertEqual(score(start_scene(cuff_8=on_arm(cs.UPPER_Y[1] - 0.03))), 0.25)   # 1 cm past the shoulder
        self.assertEqual(score(start_scene(cuff_8=on_arm(cs.UPPER_Y[1] + 0.02))), 0.0)    # clear off the end
        self.assertEqual(score(start_scene(cuff_8=on_arm(cs.FOREARM_Y[0] + 0.03))), 0.25)  # 1 cm past the hand

    def test_the_scorer_rejects_a_cuff_set_with_no_single_right_size(self):
        old = dict(cs.WIDTHS)
        try:
            cs.WIDTHS["cuff_12"] = 0.08
            with self.assertRaises(ValueError):
                score(start_scene())
        finally:
            cs.WIDTHS.clear()
            cs.WIDTHS.update(old)


class DemoPlan(unittest.TestCase):
    EXPECTED = {"correct": 1.0, "narrow": 0.0, "wide": 0.0, "elbow": 0.5, "beside": 0.0, "hold": 0.25, "nothing": 0.0}

    def test_every_waypoint_is_in_reach_and_above_the_table(self):
        for variant in self.EXPECTED:
            for dur, (x, y, z), finger, label in cs.demo_plan(variant, TOP):
                d = math.hypot(x - cs.RIGHT_BASE[0], y - cs.RIGHT_BASE[1])
                self.assertTrue(cs.REACH_M[0] <= d <= cs.REACH_M[1], (variant, label, d))
                self.assertTrue(TOP + 0.05 <= z <= TOP + 0.25, (variant, label, z - TOP))
                self.assertTrue(finger == "open" or finger == cs.GRIP_VALUE)
                self.assertGreater(dur, 0)

    def test_the_demo_fits_the_run_limit_with_room(self):
        for variant in self.EXPECTED:
            seconds = sum(s[0] for s in cs.demo_plan(variant, TOP))
            self.assertLess(seconds * 20, 600, variant)                 # well under the 1,200-step limit

    def test_the_grip_is_at_the_knob_and_the_carry_clears_the_arm_and_the_row(self):
        plan = cs.demo_plan("correct", TOP)
        grip_z = TOP + cs.SPAWN_LIFT + cs.KNOB_MID + cs.TCP_ABOVE_PAD
        knob_bottom = TOP + cs.SPAWN_LIFT + cs.LEG_H + cs.BRIDGE_T
        pad_tip = grip_z - cs.TCP_ABOVE_PAD
        self.assertGreater(pad_tip, knob_bottom + 0.01)                 # pads on the knob, not on the bridge
        self.assertLess(pad_tip, knob_bottom + cs.KNOB[2])
        carry = [s for s in plan if s[3] == "carry over the arm"][0]
        feet = carry[1][2] - (grip_z - TOP - cs.SPAWN_LIFT)            # the cuff's feet while carried
        self.assertGreater(feet, TOP + cs.LIMB_H + 0.03)
        self.assertGreater(feet, TOP + cs.HEIGHT - 0.03)               # and nearly over the other cuffs' knobs

    def test_the_demo_ends_clear_of_the_camera_except_when_holding(self):
        for variant in self.EXPECTED:
            last = cs.demo_plan(variant, TOP)[-1]
            if variant not in ("hold", "nothing"):
                self.assertEqual(last[3], "park clear of the camera", variant)
                self.assertEqual(last[2], "open")

    def test_set_down_spots_do_not_hit_the_other_cuffs(self):
        for variant, (name, tx, ty, release) in cs.VARIANTS.items():
            for other in cs.CUFF_NAMES:
                if other == name:
                    continue
                dx = abs(tx - cs.START_X) - cs.OUTER
                dy = abs(ty - cs.START_Y[other]) - (cs.WIDTHS[name] + cs.WIDTHS[other]) / 2
                self.assertTrue(dx >= 0.015 or dy >= 0.015, (variant, other))

    def test_each_variant_scores_what_it_should_if_the_cuff_ends_where_the_demo_leaves_it(self):
        for variant, expected in self.EXPECTED.items():
            if variant == "nothing":
                final = start_scene()
            else:
                name, tx, ty, release = cs.VARIANTS[variant]
                z = 0.0 if release else cs.TRAVEL_ABOVE - (cs.KNOB_MID + cs.TCP_ABOVE_PAD)
                final = start_scene(**{name: state(tx, ty, z)})
            self.assertEqual(score(final), expected, variant)


if __name__ == "__main__":
    unittest.main()

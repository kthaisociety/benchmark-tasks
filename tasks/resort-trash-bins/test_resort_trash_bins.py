"""Offline checks of task logic with stand-ins, not a simulator trial.

Run with: python3 -m unittest discover -s tasks/resort-trash-bins -p 'test_*.py'
"""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


class Primitive:
    def __init__(self, name, **kwargs):
        self.name = name
        self.__dict__.update(kwargs)


class Waypoint:
    def __init__(self, duration, pos, finger, label):
        self.duration, self.pos, self.finger, self.label = duration, pos, finger, label


def load_task():
    modules = {name: types.ModuleType(name) for name in (
        "yam_arena", "yam_arena.constants", "yam_arena.demo", "yam_arena.task_file")}
    modules["yam_arena.constants"].FINGER_CLOSED = 0.0
    modules["yam_arena.constants"].FINGER_OPEN = 1.0
    modules["yam_arena.demo"].Waypoint = Waypoint
    modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.8}
    modules["yam_arena.task_file"].Box = Primitive
    modules["yam_arena.task_file"].Cylinder = Primitive
    spec = importlib.util.spec_from_file_location("resort_task", Path(__file__).with_name("resort_trash_bins.py"))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


M = load_task()


class State:
    def __init__(self, name, destination, slot, bottom=None, moving=False, tilt_deg=0):
        spec = M.ITEMS[name]
        dims = spec[4]
        half_x = dims[0] if spec[3] == "cylinder" else dims[0] / 2
        half_y = dims[0] if spec[3] == "cylinder" else dims[1] / 2
        x, y = M._slot(destination, slot)
        bottom = M.FLOOR_Z if bottom is None else bottom
        self.low = (x - half_x, y - half_y, bottom)
        self.high = (x + half_x, y + half_y, bottom + dims[-1])
        a = math.radians(tilt_deg) / 2
        self.quat_xyzw = (math.sin(a), 0, 0, math.cos(a))
        self.moving = moving

    def resting(self):
        return not self.moving


def initial():
    return {name: State(name, spec[1], spec[2]) for name, spec in M.ITEMS.items()}


def completed():
    states = initial()
    for name, destination, slot, _ in M.MOVES:
        states[name] = State(name, destination, slot)
    return states


class TaskLogicTests(unittest.TestCase):
    def test_initial_scene_earns_zero(self):
        self.assertEqual(M.score(initial()), 0.0)

    def test_candidate_destinations_earn_full_credit(self):
        self.assertEqual(M.score(completed()), 1.0)

    def test_missing_item_loses_its_category(self):
        states = completed()
        states["glass_in_paper"] = State("glass_in_paper", "paper", 1, bottom=M.FLOOR_Z + 0.03)
        self.assertEqual(M.score(states), 0.75)

    def test_wrong_item_contaminates_its_destination(self):
        states = completed()
        states["other_in_glass"] = State("other_in_glass", "glass", 3)
        self.assertEqual(M.score(states), 0.5)

    def test_tilt_movement_and_stacking_are_not_accepted(self):
        for kwargs in ({"tilt_deg": 16}, {"moving": True}, {"bottom": M.FLOOR_Z + 0.065}):
            with self.subTest(kwargs=kwargs):
                states = completed()
                states["glass_correct"] = State("glass_correct", "glass", 0, **kwargs)
                self.assertEqual(M.score(states), 0.75)

    def test_outside_opening_is_not_accepted(self):
        states = completed()
        state = states["glass_correct"]
        state.high = (M.DESTINATIONS["glass"][0] + M.INNER / 2 + 0.001, state.high[1], state.high[2])
        self.assertEqual(M.score(states), 0.75)

    def test_planned_slots_are_unoccupied_and_moves_match_categories(self):
        occupied = {(spec[1], spec[2]): name for name, spec in M.ITEMS.items()}
        for name, destination, slot, _ in M.MOVES:
            source = next(key for key, item in occupied.items() if item == name)
            self.assertNotIn((destination, slot), occupied, name)
            self.assertEqual(M.ITEMS[name][0], destination)
            del occupied[source]
            occupied[(destination, slot)] = name
        self.assertTrue(all(M.ITEMS[name][0] == location[0] for location, name in occupied.items()))

    def test_demo_arm_plans_have_equal_duration_and_fit_budget(self):
        plans = M.demo()
        durations = [sum(waypoint.duration for waypoint in plans[side]) for side in ("left", "right")]
        self.assertAlmostEqual(*durations)
        self.assertLessEqual(max(durations) * 20, 2400)


if __name__ == "__main__":
    unittest.main()

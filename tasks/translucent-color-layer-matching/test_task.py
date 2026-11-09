"""Offline tests: no Isaac Sim, yam_arena, network, or GPU is needed."""

from itertools import combinations, permutations
from dataclasses import dataclass, field
import importlib.util
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import filter_scoring as s
import optics

TOP = 0.75
FLOOR = TOP + s.LIGHTBOX_H
H = s.PLAQUE_SIZE[2]


def pose(x, y, z, *, yaw=0.0, tilt=0.0):
    # yaw around z, then tilt around x; quaternion in xyzw order
    a, b = math.sin(tilt / 2), math.cos(tilt / 2)
    c, d = math.sin(yaw / 2), math.cos(yaw / 2)
    return SimpleNamespace(pos=(x, y, z), quat_xyzw=(a*d, a*c, b*c, b*d))


def scene(order=()):
    states = {name: pose(0.30, -0.145, FLOOR + (level + 0.5) * s.PLAQUE_SIZE[2])
              for level, name in enumerate(s.REFERENCE_PARTS)}
    for i, name in enumerate(s.FILTER_NAMES):
        states[name] = pose(0.12 + i * 0.12, 0.30, TOP + s.PLAQUE_SIZE[2] / 2)
    for level, name in enumerate(order):
        states[name] = pose(*s.STACK_XY, FLOOR + (level + 0.5) * s.PLAQUE_SIZE[2])
    return states


def ready(states):
    trial = s.Trial(TOP)
    trial.reset({name: states[name] for name in s.REFERENCE_PARTS})
    for _ in range(s.STILL_STEPS):
        trial.observe(states)
    return trial


class ScoringTests(unittest.TestCase):
    def test_every_subset_and_order_matches_the_credit_scheme(self):
        for count in range(len(s.FILTER_NAMES) + 1):
            for names in combinations(s.FILTER_NAMES, count):
                for order in permutations(names):
                    states = scene(order)
                    trial = ready(states)
                    stacked = frozenset(names)
                    if stacked in optics.MATCHING_SETS:
                        expect = 1.0
                    elif any(m <= stacked for m in optics.MATCHING_SETS):
                        expect = 0.5
                    elif stacked and any(stacked < m for m in optics.MATCHING_SETS):
                        expect = 0.25
                    else:
                        expect = 0.0
                    with self.subTest(order=order):
                        self.assertEqual(trial.score(states), expect)

    def test_unused_plaques_can_be_elsewhere_even_stacked(self):
        states = scene(("amber", "cyan"))
        states["violet"] = pose(0.45, -0.30, TOP + 0.5 * H)
        states["lime"] = pose(0.45, -0.30, TOP + 1.5 * H)
        self.assertEqual(ready(states).score(states), 1.0)

    def test_extra_touching_stack_even_if_offset_or_tilted_removes_clean_bonus(self):
        for tilt in (0.0, math.radians(20)):
            states = scene(("amber", "cyan"))
            states["violet"] = pose(s.STACK_XY[0] + 0.030, s.STACK_XY[1],
                                    FLOOR + 2.5 * H, tilt=tilt)
            self.assertEqual(ready(states).score(states), 0.5)

    def test_unsettled_extra_touching_stack_still_removes_clean_bonus(self):
        states = scene(("amber", "cyan", "violet"))
        trial = ready(states)
        states["violet"] = pose(s.STACK_XY[0] + 0.003, s.STACK_XY[1], FLOOR + 2.5 * H)
        trial.observe(states)
        self.assertEqual(trial.score(states), 0.5)

    def test_extra_separate_from_stack_does_not_remove_clean_bonus(self):
        states = scene(("amber", "cyan"))
        # A neighbouring plaque at the same height, 1 mm clear of the stack.
        states["violet"] = pose(s.STACK_XY[0] + 0.061, s.STACK_XY[1], FLOOR + 0.5 * H)
        self.assertEqual(ready(states).score(states), 1.0)

    def test_no_support_no_credit_and_no_floating_upper_credit(self):
        clear = s.SUPPORT_GAP + s.TOUCH_Z + 0.002  # clear of both tolerances
        states = scene(("amber",))
        states["amber"] = pose(*s.STACK_XY, FLOOR + 0.5 * H + clear)
        self.assertEqual(ready(states).score(states), 0.0)
        states = scene(("amber", "cyan"))
        states["cyan"] = pose(*s.STACK_XY, FLOOR + 1.5 * H + clear)
        # cyan floats clear of the stack: amber alone, with nothing attached
        self.assertEqual(ready(states).score(states), 0.25)

    def test_support_gap_tolerance_boundary(self):
        for order in (("amber", "cyan"), ("cyan", "amber")):
            # inside the 3 mm tolerance the upper gel counts
            states = scene(order)
            states[order[1]] = pose(*s.STACK_XY, FLOOR + 1.5 * H + s.SUPPORT_GAP - 0.0001)
            with self.subTest(order=order, gap="inside"):
                self.assertEqual(ready(states).score(states), 1.0)
            # Beyond it the upper gel is neither supported nor (since
            # SUPPORT_GAP equals TOUCH_Z) attached, so only the lower one counts.
            states = scene(order)
            states[order[1]] = pose(*s.STACK_XY, FLOOR + 1.5 * H + s.SUPPORT_GAP + 0.0001)
            with self.subTest(order=order, gap="outside"):
                self.assertEqual(ready(states).score(states), 0.25)
        states = scene(("amber", "cyan"))
        # Includes the 0.2 mm raised outline under the bottom plaque.
        for level, name in enumerate(("amber", "cyan")):
            states[name] = pose(*s.STACK_XY, FLOOR + (level + 0.5) * H + 0.0002)
        self.assertEqual(ready(states).score(states), 1.0)

    def test_pose_alone_cannot_prove_release_within_the_tolerance(self):
        states = scene(("amber", "cyan"))
        states["cyan"] = pose(*s.STACK_XY, FLOOR + 1.5 * H + 0.0004)
        # The adapter exposes no held/contact flag: this pose has the same
        # score whether a gripper is holding it or the physics supports it.
        # The 3 mm support tolerance widens this; see README.
        self.assertEqual(ready(states).score(states), 1.0)

    def test_placement_and_flatness(self):
        states = scene(("amber",))
        states["amber"] = pose(s.STACK_XY[0] + s.CENTRE_TOL + 0.001,
                               s.STACK_XY[1], FLOOR + 0.5 * H)
        self.assertEqual(ready(states).score(states), 0.0)
        states["amber"] = pose(*s.STACK_XY, FLOOR + 0.5 * H, tilt=math.radians(12))
        self.assertEqual(ready(states).score(states), 0.0)
        states["amber"] = pose(*s.STACK_XY, FLOOR + 0.5 * H, yaw=math.radians(45))
        self.assertEqual(ready(states).score(states), 0.25)
        states["amber"] = pose(*s.STACK_XY, FLOOR + 0.5 * H, tilt=math.pi)
        self.assertEqual(ready(states).score(states), 0.0)

    def test_stack_centres_must_stay_within_the_light_tolerance(self):
        # Two gels offset to opposite limits still share a central patch of
        # the lit window: 22 mm aperture half-width minus the centre tolerance
        # each way. Its corners stay inside the 22 mm inscribed radius, so the
        # patch survives any yaw.
        shared = 0.022 - s.CENTRE_TOL
        self.assertGreater(shared, 0.0)
        self.assertLess(math.hypot(shared, shared), 0.022)
        for order in (("amber", "cyan"), ("cyan", "amber")):
            states = scene(order)
            for level, name in enumerate(order):
                offset = s.CENTRE_TOL if level == 0 else -s.CENTRE_TOL
                states[name] = pose(s.STACK_XY[0] + offset, s.STACK_XY[1] - offset,
                                    FLOOR + (level + 0.5) * H, yaw=math.pi / 4)
            self.assertEqual(ready(states).score(states), 1.0)
            states[order[1]] = pose(s.STACK_XY[0] + s.CENTRE_TOL + 0.0001, s.STACK_XY[1] - 0.005,
                                    FLOOR + 1.5 * H)
            self.assertEqual(ready(states).score(states), 0.5)
            for level, name in enumerate(order):
                states[name] = pose(s.STACK_XY[0] + s.CENTRE_TOL + 0.005, s.STACK_XY[1],
                                    FLOOR + (level + 0.5) * H)
            self.assertEqual(ready(states).score(states), 0.0)

    def test_stillness_requires_twenty_consistent_samples(self):
        states = scene(("amber",))
        trial = s.Trial(TOP)
        trial.reset({n: states[n] for n in s.REFERENCE_PARTS})
        for _ in range(s.STILL_STEPS - 1):
            trial.observe(states)
        self.assertEqual(trial.score(states), 0.0)
        trial.observe(states)
        self.assertEqual(trial.score(states), 0.25)
        states["amber"] = pose(s.STACK_XY[0] + 0.005, s.STACK_XY[1], FLOOR + 0.5 * H)
        trial.observe(states)
        self.assertEqual(trial.score(states), 0.0)
        for _ in range(s.STILL_STEPS - 1):
            trial.observe(states)
        self.assertEqual(trial.score(states), 0.25)

    def test_reference_displacement_is_latched_until_reset(self):
        states = scene(("amber", "cyan"))
        trial = ready(states)
        reference = states[s.REFERENCE]
        states[s.REFERENCE] = pose(reference.pos[0] + s.REF_POSITION - 0.00001,
                                   reference.pos[1], reference.pos[2])
        trial.observe(states)
        self.assertEqual(trial.score(states), 1.0)
        states[s.REFERENCE] = pose(reference.pos[0] + s.REF_POSITION + 0.00001,
                                   reference.pos[1], reference.pos[2])
        trial.observe(states)
        states[s.REFERENCE] = reference
        trial.observe(states)
        self.assertEqual(trial.score(states), 0.0)
        for _ in range(2):
            trial.reset({n: states[n] for n in s.REFERENCE_PARTS})
            # Every reset clears both the violation and prior stillness samples.
            self.assertEqual(trial.score(states), 0.0)
            for _ in range(s.STILL_STEPS):
                trial.observe(states)
            self.assertEqual(trial.score(states), 1.0)

    def test_reference_rotation_and_quaternion_sign(self):
        states = scene(("amber", "cyan"))
        trial = ready(states)
        ref = states[s.REFERENCE]
        states[s.REFERENCE] = SimpleNamespace(pos=ref.pos, quat_xyzw=(0, 0, 0, -1))
        trial.observe(states)
        self.assertEqual(trial.score(states), 1.0)
        states[s.REFERENCE] = pose(*ref.pos, yaw=s.REF_ROTATION + math.radians(0.1))
        trial.observe(states)
        states[s.REFERENCE] = ref
        self.assertEqual(trial.score(states), 0.0)

    def test_invalid_reference_fails_closed(self):
        states = scene(("amber", "cyan"))
        trial = ready(states)
        states[s.REFERENCE] = pose(float("nan"), 0, 0)
        trial.observe(states)
        self.assertEqual(trial.score(states), 0.0)


class OpticsTests(unittest.TestCase):
    def test_all_nominal_combinations_and_both_orders(self):
        self.assertIs(optics.REQUIRED, s.REQUIRED)
        self.assertEqual(set(optics.FILTER_TINTS), set(s.FILTER_NAMES))
        self.assertEqual(optics.REFERENCE_TINT, optics.nominal_tint(s.REQUIRED))
        self.assertEqual(optics.nominal_tint(("amber", "cyan")), optics.REFERENCE_TINT)
        self.assertEqual(optics.nominal_tint(("cyan", "amber")), optics.REFERENCE_TINT)
        alternatives = [optics.difference(optics.nominal_tint(c), optics.REFERENCE_TINT)
                        for c in optics.subsets() if set(c) != set(optics.REQUIRED)]
        self.assertAlmostEqual(min(alternatives), 0.17)
        self.assertGreater(min(alternatives), optics.MATCH_TOLERANCE)

    def test_image_roi_compares_actual_pixels(self):
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("Pillow is not installed")
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "example.png"
            image = Image.new("RGB", (8, 4), (80, 150, 100))
            image.save(path)
            a, b, delta, clipped = optics.image_comparison(path, (0, 0, 4, 4), (4, 0, 4, 4))
            self.assertEqual(a, b)
            self.assertEqual(delta, 0)
            self.assertEqual(clipped, (0, 0))
            with self.assertRaises(ValueError):
                optics.image_comparison(path, (0, 0, 4, 4), (6, 0, 4, 4))
            with self.assertRaises(ValueError):
                optics.image_comparison(path, (0, 0, 3, 4), (4, 0, 4, 4))

class AdapterTests(unittest.TestCase):
    def test_pose_adapter_and_callback_order_with_stubs(self):
        # Exercise the real adapter with raw 2.x and 3.x simulator-shaped poses.
        import numpy as np

        @dataclass(frozen=True)
        class FakeBox:
            name: str

        @dataclass(frozen=True)
        class FakeState:
            pos: object
            quat_xyzw: object
            lin_vel: object
            ang_vel: object
            low: object
            high: object

        arena = ModuleType("yam_arena")
        arena.__path__ = []
        task_file = ModuleType("yam_arena.task_file")
        task_file.Box, task_file.ObjectState = FakeBox, FakeState
        arena.task_file = task_file
        spec = importlib.util.spec_from_file_location("filter_partkit_adapter_test", Path(__file__).with_name("partkit.py"))
        kit = importlib.util.module_from_spec(spec)

        def tensor(value):
            return SimpleNamespace(detach=lambda: SimpleNamespace(cpu=lambda: SimpleNamespace(
                numpy=lambda: np.asarray(value))))

        class Scene(dict):
            env_origins = [tensor([1.0, 2.0, 3.0])]

        modules = {"yam_arena": arena, "yam_arena.task_file": task_file, spec.name: kit}
        with mock.patch.dict(sys.modules, modules):
            try:
                spec.loader.exec_module(kit)
                factory = type("Factory", (), {"resolve_class": classmethod(lambda cls: None)})
                self.assertEqual(kit._isaac_quaternion_order(factory), "xyzw")
                for order, method in (("wxyz", "write_root_pose_to_sim"),
                                      ("xyzw", "write_root_pose_to_sim_index")):
                    for yaw, tilt, flat in ((0.0, 0.0, True), (math.pi / 2, 0.0, True),
                                            (0.0, math.radians(12), False)):
                        with self.subTest(order=order, yaw=yaw, tilt=tilt):
                            expected = pose(0.3, 0.055, FLOOR + H / 2, yaw=yaw, tilt=tilt)
                            q = expected.quat_xyzw
                            raw = (q[3], *q[:3]) if order == "wxyz" else q
                            data = SimpleNamespace(root_link_pose_w=tensor([[1.3, 2.055, 3 + FLOOR + H / 2, *raw]]),
                                                   root_vel_w=tensor([[0.0] * 6]))
                            asset = SimpleNamespace(data=data, **{method: lambda: None})
                            environment = SimpleNamespace(unwrapped=SimpleNamespace(scene=Scene(p=asset)))
                            state = kit._part_state(environment, SimpleNamespace(name="p", size=s.PLAQUE_SIZE))
                            np.testing.assert_allclose(state.pos, expected.pos, atol=1e-12)
                            np.testing.assert_allclose(state.quat_xyzw, q, atol=1e-12)
                            self.assertEqual(s._geometry(state)["flat"], flat)
                with self.assertRaisesRegex(RuntimeError, "Unrecognized"):
                    kit._isaac_quaternion_order(object())

                events = []
                class Embodiment:
                    _env = "environment"
                    def _task_step(self):
                        events.append("base step")
                    def reset(self):
                        events.append("base reset")
                kit.on_step(lambda env: events.append(("step", env)))
                kit.on_reset(lambda env: events.append(("reset", env)))
                kit._patch_embodiment(SimpleNamespace(YamArenaEmbodiment=Embodiment))
                Embodiment().reset()
                Embodiment()._task_step()
                self.assertEqual(events, ["base reset", ("reset", "environment"),
                                          ("step", "environment"), "base step"])
            finally:
                sys.meta_path[:] = [finder for finder in sys.meta_path if not isinstance(finder, kit._Finder)]


class TaskWiringTests(unittest.TestCase):
    def test_scene_and_callbacks_with_minimal_arena_stubs(self):
        # Checks Python wiring, not physics, rendering, or real simulator APIs.
        @dataclass(frozen=True)
        class FakePart:
            name: str
            pos: tuple
            boxes: tuple
            mass: float
            meta: dict = field(default_factory=dict)

        def make_part(name, pos, boxes, mass, **_):
            return FakePart(name, pos, tuple(boxes), mass)

        def fake_box(name, **kwargs):
            return SimpleNamespace(name=name, **kwargs)

        kit = ModuleType("partkit")
        kit.part = make_part
        kit.on_reset = kit.on_step = lambda fn: fn
        arena = ModuleType("yam_arena")
        arena.__path__ = []
        constants = ModuleType("yam_arena.constants")
        constants.FINGER_OPEN = 0.04
        demo = ModuleType("yam_arena.demo")
        demo.Waypoint = lambda *args: args
        task_file = ModuleType("yam_arena.task_file")
        task_file.TABLE_TOP = {"station": TOP}
        task_file.Box = fake_box
        modules = {"partkit": kit, "yam_arena": arena, "yam_arena.constants": constants,
                   "yam_arena.demo": demo, "yam_arena.task_file": task_file}
        with mock.patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location("filter_task_stub", Path(__file__).with_name("task.py"))
            task = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(task)
            metadata = json.loads(Path(__file__).with_name("task.json").read_text())
            self.assertEqual(task.INSTRUCTION, metadata["instruction"])
            parts = {obj.name: obj for obj in task.OBJECTS if isinstance(obj, FakePart)}
            self.assertEqual(set(parts), set(s.FILTER_NAMES) | set(s.REFERENCE_PARTS))
            self.assertGreater(parts[s.REFERENCE].mass, 0)
            self.assertEqual(len(parts[s.REFERENCE].boxes), 5)
            outline = [obj for obj in task.OBJECTS if obj.name.startswith("outline_")]
            self.assertEqual(len(outline), 4)
            for axis in (0, 1):
                low = min(obj.pos[axis] - obj.size[axis] / 2 for obj in outline)
                high = max(obj.pos[axis] + obj.size[axis] / 2 for obj in outline)
                self.assertAlmostEqual(high - low, s.PLAQUE_SIZE[axis])
                self.assertAlmostEqual((high + low) / 2, s.STACK_XY[axis])
            self.assertTrue(all(abs(obj.pos[2] + obj.size[2] / 2 - FLOOR - 0.0002) < 1e-10
                                for obj in outline))
            self.assertTrue(task.demo()["left"])
            self.assertFalse(hasattr(task, "success"))

            states = scene(("amber", "cyan"))
            environment = SimpleNamespace(states=states)
            task._states = lambda env: env.states
            task._install_windows = lambda env: None
            task._reset(environment)
            for _ in range(s.STILL_STEPS):
                task._step(environment)
            self.assertEqual(task.score(states), 1.0)
            reference = states[s.REFERENCE]
            states[s.REFERENCE] = pose(reference.pos[0] + s.REF_POSITION + 0.001, *reference.pos[1:])
            task._step(environment)
            states[s.REFERENCE] = reference
            task._step(environment)
            self.assertEqual(task.score(states), 0.0)
            task._reset(environment)
            for _ in range(s.STILL_STEPS):
                task._step(environment)
            self.assertEqual(task.score(states), 1.0)


if __name__ == "__main__":
    unittest.main()

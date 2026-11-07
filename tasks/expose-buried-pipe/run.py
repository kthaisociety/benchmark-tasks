"""Run the buried-pipe trainer on the Robocurve Isaac Lab-Arena station.

Wraps the ``inspect-robots`` CLI: authors the compound soil box, pipe, spoil board, holder and spade and the loose
soil cubes, lets the scripted demo tilt the spade, and scores every control step with ``scoring.Judge``.
"""

import argparse
import atexit
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

import scoring
import task as fixture

BALL_GLOB = "/World/envs/env_*/soil/cube_*"


def install_tilt():
    """Let a ``TiltWaypoint`` pitch the straight-down grip about world y (the stock plan only yaws)."""
    from yam_arena import demo, ik
    from yam_arena.urdf import axis_angle_matrix

    def rotation(w):
        rot = demo.grip_rotation(w.yaw)
        pitch = getattr(w, "pitch", 0.0)
        return rot if pitch == 0.0 else axis_angle_matrix(np.array([0.0, 1.0, 0.0]), pitch) @ rot

    def target(self, t):
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal, rot = np.asarray(w.pos, dtype=float), rotation(w)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return (
                    prev_pos + s * (goal - prev_pos),
                    ik.slerp_matrix(prev_rot, rot, s),
                    prev_finger + s * (w.finger - prev_finger),
                    w.label,
                )
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return prev_pos, prev_rot, prev_finger, "done"

    demo.ArmPlan.target = target


def install_scene():
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original = scene.task_object

    def material(stage, path, static, dynamic):
        mat = UsdShade.Material.Define(stage, path)
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        return mat

    def collide(prim, mat, offset=0.001):
        UsdPhysics.CollisionAPI.Apply(prim)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision.CreateContactOffsetAttr(offset)
        collision.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")

    def colour(geom, rgb):
        geom.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*scene.linear_rgb(rgb))])

    def task_object(spec):
        obj = original(spec)
        if spec.name not in fixture.PARTS:
            return obj

        @clone
        def spawn(path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            root = UsdGeom.Xform.Define(stage, path)
            origin = translation if translation is not None else spec.pos
            root.AddTranslateOp().Set(Gf.Vec3d(*origin))
            prim = root.GetPrim()
            if spec.name == "soil":
                mat = material(stage, f"{path}/soil_material", 0.8, 0.6)
                for i, centre in enumerate(fixture.FILL):
                    cube = UsdGeom.Cube.Define(stage, f"{path}/cube_{i:03d}")
                    cube.CreateSizeAttr(fixture.CUBE)
                    local = [centre[k] - origin[k] for k in range(3)]
                    cube.AddTranslateOp().Set(Gf.Vec3d(*local))
                    colour(cube, fixture.SOIL)
                    bp = cube.GetPrim()
                    UsdPhysics.RigidBodyAPI.Apply(bp)
                    UsdPhysics.MassAPI.Apply(bp).CreateMassAttr(fixture.CUBE_MASS)
                    body = PhysxSchema.PhysxRigidBodyAPI.Apply(bp)
                    body.CreateAngularDampingAttr(0.5)
                    body.CreateLinearDampingAttr(0.2)
                    body.CreateSolverPositionIterationCountAttr(8)
                    body.CreateSolverVelocityIterationCountAttr(2)
                    collide(bp, mat, 0.001)
                return prim
            if spec.free:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateSolverPositionIterationCountAttr(32)
                body.CreateSolverVelocityIterationCountAttr(8)
                body.CreateSleepThresholdAttr(0.0)
                mat = material(stage, f"{path}/contact_material", 1.2, 1.0)
            else:
                mat = material(stage, f"{path}/contact_material", 0.8, 0.6)
            for part in fixture.PARTS[spec.name]:
                if part.shape == "cylinder_x":
                    geom = UsdGeom.Cylinder.Define(stage, f"{path}/{part.name}")
                    geom.CreateAxisAttr("X")
                    geom.CreateHeightAttr(part.size[0])
                    geom.CreateRadiusAttr(part.size[1])
                    geom.AddTranslateOp().Set(Gf.Vec3d(*part.center))
                else:
                    geom = UsdGeom.Cube.Define(stage, f"{path}/{part.name}")
                    geom.CreateSizeAttr(1.0)
                    geom.AddTranslateOp().Set(Gf.Vec3d(*part.center))
                    geom.AddScaleOp().Set(Gf.Vec3f(*part.size))
                colour(geom, part.color)
                if part.collides:
                    collide(geom.GetPrim(), mat, 0.002 if spec.free else 0.001)
            return prim

        obj.spawner_cfg.func = spawn
        return Object(
            name=spec.name,
            object_type=obj.object_type,
            spawner_cfg=obj.spawner_cfg,
            initial_pose=obj.initial_pose,
        )

    scene.task_object = task_object


def to_numpy(arr):
    import warp as wp

    if isinstance(arr, wp.array):
        return wp.to_torch(arr).detach().cpu().numpy()
    return arr.detach().cpu().numpy() if hasattr(arr, "detach") else np.asarray(arr)


def install(trace_dir):
    from inspect_robots.types import StepResult
    from yam_arena import inspect_embodiment as arena
    from yam_arena.runtime import tcp_world

    cls = arena.YamArenaEmbodiment
    original_ensure = cls._ensure_sim
    original_reset = cls.reset
    scene_installed = False

    def ensure(self, goal):
        nonlocal scene_installed
        if arena._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            opts = dict(headless=self.headless, device=self.device, enable_cameras=True)
            if not self.headless:
                os.environ["DISPLAY"] = arena.window_display(os.environ)
                opts["visualizer"] = ["kit"]
            arena._APP = AppLauncher(**opts).app
            atexit.register(arena._close_app_at_exit)
        if not scene_installed:
            install_scene()
            scene_installed = True
        return original_ensure(self, goal)

    def balls_view(self):
        view = getattr(self, "_balls_view", None)
        if view is None:
            from isaaclab.sim import SimulationContext

            view = SimulationContext.instance().physics_sim_view.create_rigid_body_view(BALL_GLOB)
            if view.count != len(fixture.FILL):
                raise RuntimeError(f"found {view.count} soil cubes, expected {len(fixture.FILL)}")
            self._balls_view = view
        return view

    def put_soil_back(self):
        """Return every soil ball to its starting place, at rest (the arena's own reset leaves them)."""
        import torch
        import warp as wp

        view = balls_view(self)
        origin = self._base.scene.env_origins[0].detach().cpu().numpy()
        pose = np.zeros((len(fixture.FILL), 7), dtype=np.float32)
        pose[:, :3] = np.asarray(fixture.FILL) + origin
        pose[:, 6] = 1.0
        device = self._base.device
        idx = wp.from_torch(torch.arange(len(fixture.FILL), dtype=torch.int32, device=device))
        view.set_transforms(wp.from_torch(torch.tensor(pose, device=device)), indices=idx)
        zeros = torch.zeros((len(fixture.FILL), 6), dtype=torch.float32, device=device)
        view.set_velocities(wp.from_torch(zeros), indices=idx)

    def reset(self, scene, *, seed=None):
        self.judge = scoring.Judge(control_hz=self.info.control_hz)
        self.trace_index = getattr(self, "trace_index", 0) + 1
        self._ensure_sim(None)
        if getattr(self, "_episodes", 0) > 0:
            put_soil_back(self)
        self._episodes = getattr(self, "_episodes", 0) + 1
        return original_reset(self, scene, seed=seed)

    def step(self):
        base = self._base
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        spade = base.scene["spade"].data.root_link_pose_w.torch[0].detach().cpu().numpy()
        spade_pose = (spade[:3] - origin, spade[3:7])
        view = balls_view(self)
        balls = to_numpy(view.get_transforms()).reshape(-1, 7)[:, :3] - origin
        speeds = np.linalg.norm(to_numpy(view.get_velocities()).reshape(-1, 6)[:, :3], axis=1)
        info = self.judge.step(spade_pose, balls, speeds)
        success = info["success"]
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            tcp = {arm: [float(v) for v in tcp_world(base.scene[arm])] for arm in arena.ARMS}
            with (trace_dir / f"dig-{self.trace_index}.jsonl").open("a") as stream:
                print(json.dumps({
                    **{k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in info.items()},
                    "time_s": self._t,
                    "tcp_m": tcp,
                    "spade_pos_m": [float(v) for v in spade_pose[0]],
                    "spade_quat_xyzw": [float(v) for v in spade_pose[1]],
                    "cubes_in_box": int(scoring.over_pipe(balls).sum()),
                    "cubes_on_board": int(on_board(balls).sum()),
                    "cubes_off_table": int((balls[:, 2] < fixture.TOP - 0.05).sum()),
                }), file=stream)
            if self._t < 1.5 / self.info.control_hz or success:
                np.save(trace_dir / f"balls-{self.trace_index}-{'end' if success else 'start'}.npy", balls)
        return StepResult(
            observation=self._observation(),
            terminated=success,
            termination_reason="success" if success else None,
            info={k: (float(v) if isinstance(v, (float, np.floating)) and math.isfinite(v) else v)
                  for k, v in info.items() if not (isinstance(v, float) and not math.isfinite(v))},
        )

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, step


def on_board(balls):
    b = np.asarray(balls)
    return b[:, 0] < fixture.BOX_X[0] - fixture.WALL


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    if args.log_dir:
        sys.argv += ["--log-dir", str(args.log_dir)]
    install_tilt()
    install(args.log_dir)

    from inspect_robots.cli import main

    main()

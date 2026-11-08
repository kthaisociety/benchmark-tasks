"""Run the passing-loop fixture on the Robocurve Isaac Lab-Arena station."""

import argparse
import atexit
import json
import math
import os
from pathlib import Path
import sys

import layout as L
import scoring
import task as fixture

LEVER_C = 0.004  # N m s/rad of pivot friction; the lever's own weight holds it on either stop
BLADE_K, BLADE_C, BLADE_MAX = 0.08, 0.004, 0.06  # the rod's pull on each blade
GATE_FORCE = 0.6  # N
FILTERED = ("blade_a", "blade_b", "gate", "lever")


def install_scene():
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena import scene

    original = scene.task_object

    def task_object(spec):
        obj = original(spec)
        if spec.name not in fixture.PARTS:
            return obj
        physics_cfg = fixture.BODIES[spec.name]

        @clone
        def spawn(path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            root = UsdGeom.Xform.Define(stage, path)
            origin = translation if translation is not None else spec.pos
            root.AddTranslateOp().Set(Gf.Vec3d(*origin))
            prim = root.GetPrim()
            if spec.free:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateDisableGravityAttr(not physics_cfg.gravity)
                body.CreateSolverPositionIterationCountAttr(32)
                body.CreateSolverVelocityIterationCountAttr(8)
                body.CreateSleepThresholdAttr(0.0)
                body.CreateLinearDampingAttr(physics_cfg.damping[0])
                body.CreateAngularDampingAttr(physics_cfg.damping[1])

            mat = UsdShade.Material.Define(stage, f"{path}/contact_material")
            material = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            material.CreateStaticFrictionAttr(physics_cfg.friction[0])
            material.CreateDynamicFrictionAttr(physics_cfg.friction[1])
            material.CreateRestitutionAttr(0.0)
            for part in fixture.PARTS[spec.name]:
                if part.shape == "sphere":
                    geom = UsdGeom.Sphere.Define(stage, f"{path}/{part.name}")
                    geom.CreateRadiusAttr(part.size[0] / 2)
                    geom.AddTranslateOp().Set(Gf.Vec3d(*part.center))
                else:
                    geom = UsdGeom.Cube.Define(stage, f"{path}/{part.name}")
                    geom.CreateSizeAttr(1.0)
                    geom.AddTranslateOp().Set(Gf.Vec3d(*part.center))
                    if part.yaw:
                        geom.AddRotateZOp().Set(math.degrees(part.yaw))
                    if part.pitch:
                        geom.AddRotateYOp().Set(math.degrees(part.pitch))
                    geom.AddScaleOp().Set(Gf.Vec3f(*part.size))
                geom.CreateDisplayColorPrimvar().Set([Gf.Vec3f(*scene.linear_rgb(part.color))])
                if part.collides:
                    shape = geom.GetPrim()
                    UsdPhysics.CollisionAPI.Apply(shape)
                    collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
                    collision.CreateContactOffsetAttr(0.001)
                    collision.CreateRestOffsetAttr(0.0)
                    UsdShade.MaterialBindingAPI.Apply(shape).Bind(mat, materialPurpose="physics")

            if spec.name in FILTERED:
                # The blades turn in gaps in the track's walls, and the gate and lever sit on the track:
                # they never touch the track itself, only the ball and the grippers.
                pairs = UsdPhysics.FilteredPairsAPI.Apply(prim)
                pairs.CreateFilteredPairsRel().AddTarget(path.rsplit("/", 1)[0] + "/track")

            guide = fixture.GUIDES.get(spec.name)
            if guide is not None:
                kind = UsdPhysics.PrismaticJoint if guide.kind == "prismatic" else UsdPhysics.RevoluteJoint
                joint = kind.Define(stage, f"{path}/guide")
                joint.CreateBody1Rel().SetTargets([root.GetPath()])
                joint.CreateLocalPos0Attr(Gf.Vec3f(*origin))
                joint.CreateAxisAttr(guide.axis)
                low, high = guide.limits
                if guide.kind == "revolute":
                    low, high = math.degrees(low), math.degrees(high)
                joint.CreateLowerLimitAttr(low)
                joint.CreateUpperLimitAttr(high)
            return prim

        obj.spawner_cfg.func = spawn
        return Object(
            name=spec.name,
            object_type=obj.object_type,
            spawner_cfg=obj.spawner_cfg,
            initial_pose=obj.initial_pose,
        )

    scene.task_object = task_object


def clamp(value, limit):
    return max(-limit, min(limit, value))


def blade_target(lever_angle, normal, reverse):
    return normal + scoring.lever_travel(lever_angle) * (reverse - normal)


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

    def state(base, name):
        data = base.scene[name].data
        pose = data.root_link_pose_w.torch[0]
        vel = data.root_link_vel_w.torch[0]
        origin = base.scene.env_origins[0]
        return (pose[:3] - origin).tolist(), pose[3:7].tolist(), vel[:3].tolist(), vel[3:6].tolist()

    def lever_angle(quat):
        qx, qy, qz, qw = quat
        return L.LEVER_NORMAL + 2.0 * math.atan2(qy, qw)

    def blade_angle(quat):
        qx, qy, qz, qw = quat
        return 2.0 * math.atan2(qz, qw)

    def reset(self, scene, *, seed=None):
        self.run_state = scoring.Run()
        self._last_torques = {}
        self.trace_index = getattr(self, "trace_index", 0) + 1
        self._ensure_sim(None)
        base = self._base
        if not getattr(base.scene, "_passing_loop_drive", False):
            import warp as wp

            torch = self._torch
            loads = {}
            for name in ("ball", "lever", "blade_a", "blade_b", "gate"):
                force = torch.zeros((1, 3), dtype=torch.float32, device=base.device)
                torque = torch.zeros_like(force)
                loads[name] = (force, torque, wp.from_torch(force), wp.from_torch(torque))
            original_write = base.scene.write_data_to_sim

            def push(name):
                force, torque, force_wp, torque_wp = loads[name]
                target = base.scene[name]
                target.root_view.apply_forces_and_torques_at_position(
                    force_data=force_wp, torque_data=torque_wp, position_data=None,
                    indices=target._ALL_INDICES, is_global=True,
                )

            def write():
                original_write()
                released = self._t >= L.RELEASE_S
                # The ball: a steady drive along the line, with speed-proportional drag.
                _, _, v, _ = state(base, "ball")
                force = loads["ball"][0]
                force[0, 0] = -L.DRAG * v[0]
                force[0, 1] = (-L.DRAG * L.BALL_SPEED if released else 0.0) - L.DRAG * v[1]
                push("ball")
                # The gate lifts at the release time.
                loads["gate"][0][0, 2] = GATE_FORCE if released else -GATE_FORCE
                push("gate")
                # The lever has no spring: past centre its weight takes it onto that stop.
                _, q, _, w = state(base, "lever")
                angle = lever_angle(q)
                loads["lever"][1][0, 1] = -LEVER_C * w[1]
                push("lever")
                # The rod drives both blades from the lever.
                for name, normal, reverse in (("blade_a", L.A_NORMAL, L.A_REVERSE),
                                              ("blade_b", L.B_NORMAL, L.B_REVERSE)):
                    _, qb, _, wb = state(base, name)
                    error = blade_target(angle, normal, reverse) - blade_angle(qb)
                    loads[name][1][0, 2] = clamp(BLADE_K * error - BLADE_C * wb[2], BLADE_MAX)
                    self._last_torques[name] = loads[name][1][0, 2].item()
                    push(name)

            base.scene.write_data_to_sim = write
            base.scene._passing_loop_drive = True
        return original_reset(self, scene, seed=seed)

    def step(self):
        base = self._base
        ball, _, ball_v, _ = state(base, "ball")
        _, lever_q, _, _ = state(base, "lever")
        _, blade_a_q, _, _ = state(base, "blade_a")
        _, blade_b_q, _, _ = state(base, "blade_b")
        gate, _, _, _ = state(base, "gate")
        tcp = {arm: [float(v) for v in tcp_world(base.scene[arm])] for arm in arena.ARMS}
        lever = lever_angle(lever_q)
        run = self.run_state
        value = run.step(scoring.Sample(ball=tuple(ball), lever=lever, tcp=tuple(map(tuple, tcp.values()))))
        info = {
            "score": value,
            "success": run.merged and not run.fault,
            "ball_m": ball,
            "lever_travel": scoring.lever_travel(lever),
            "entered_loop": run.entered_loop,
            "in_sand_drag": run.in_pit,
            "merged": run.merged,
            "fault": run.fault,
        }
        if trace_dir is not None:
            trace_dir.mkdir(parents=True, exist_ok=True)
            with (trace_dir / f"trace-{self.trace_index}.jsonl").open("a") as stream:
                print(json.dumps({
                    **info,
                    "time_s": self._t,
                    "ball_v_m_s": ball_v,
                    "lever_deg": math.degrees(lever),
                    "blade_a_deg": math.degrees(blade_angle(blade_a_q)),
                    "blade_b_deg": math.degrees(blade_angle(blade_b_q)),
                    "gate_m": gate,
                    "torque_a": float(self._last_torques.get("blade_a", 0.0)),
                    "tcp_m": tcp,
                }), file=stream)
        done = info["success"] or run.fault is not None
        return StepResult(
            observation=self._observation(),
            terminated=done,
            termination_reason="success" if info["success"] else "fault" if run.fault else None,
            info=info,
        )

    cls._ensure_sim, cls.reset, cls._task_step = ensure, reset, step


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--log-dir", type=Path)
    args, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    if args.log_dir:
        sys.argv += ["--log-dir", str(args.log_dir)]
    install(args.log_dir)

    from inspect_robots.cli import main

    main()

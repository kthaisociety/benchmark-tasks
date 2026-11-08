"""Run inspect-robots with the wheelchair (chair.build, a jointed body) spawned in place of the `chair` placeholder
box, the chair's pose and both tool centre points added to what score() sees, and the scorer reset at every
trial. Same command line as inspect-robots. With --log-dir, a per-step trace goes to <log-dir>/trace-<n>.jsonl."""

import atexit
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import chair as C  # noqa: E402

NAME = "chair"


def install(trace_dir=None):
    import numpy as np
    from yam_arena import inspect_embodiment as arena
    from yam_arena.task_file import ObjectState

    cls = arena.YamArenaEmbodiment
    original_ensure, original_step, original_reset = cls._ensure_sim, cls._task_step, cls.reset
    patched = {"done": False}

    def install_scene():
        import isaaclab.sim as sim_utils
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose

        original = scene.task_object

        def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
            import omni.usd
            from pxr import Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()
            x = UsdGeom.Xform.Define(stage, prim_path)
            x.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
            x.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            return C.build(stage, prim_path)

        def task_object(spec):
            if spec.name != NAME:
                return original(spec)
            cfg = sim_utils.CuboidCfg(size=(0.01, 0.01, 0.01))
            cfg.func = spawn
            return Object(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg,
                          initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

        scene.task_object = task_object

    def ensure(self, goal):
        if arena._APP is None:
            from isaaclab.app import AppLauncher

            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            arena._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(arena._close_app_at_exit)
        if not patched["done"]:
            install_scene()
            patched["done"] = True
        return original_ensure(self, goal)

    def step(self):
        from yam_arena import scene
        from yam_arena.runtime import tcp_world

        base = self._base
        arr = lambda t: t.detach().cpu().numpy().astype(float)  # noqa: E731
        origin = arr(base.scene.env_origins[0])
        world = base.arena_world
        pose = arr(world.get_pose_w(NAME)[0])
        p = pose[:3] - origin
        extra = {NAME: ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=arr(world.get_root_linear_velocity_w(NAME)[0]),
                                   ang_vel=arr(world.get_root_angular_velocity_w(NAME)[0]), low=p, high=p)}
        for arm in arena.ARMS:
            t = np.asarray(tcp_world(base.scene[arm]), float) - origin
            z = np.zeros(3)
            extra[arm.split("_")[0] + "_tcp"] = ObjectState(pos=t, quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=z, ang_vel=z, low=t, high=t)
        original_states = scene.object_states

        def object_states(env, names):
            return {**original_states(env, [n for n in names if n != NAME]), **extra}

        scene.object_states = object_states
        try:
            result = original_step(self)
        finally:
            scene.object_states = original_states
        m = self.task.module
        if trace_dir is not None:
            import scoring

            states = {**extra}
            d = original_states(self._env, ["dummy"])["dummy"]
            states["dummy"] = d
            r = lambda v: [round(float(a), 4) for a in v]  # noqa: E731
            w = scoring.wheels(states)
            row = {"time_s": round(self._t, 3), "score": result.info.get("score"), "fault": m.fault(),
                   "axle": r(p), "pitch_deg": round(math.degrees(scoring.pitch(states)), 2),
                   "roll_deg": round(math.degrees(scoring.roll(states)), 2), "dummy": r(d.pos),
                   "wheels": {n: r(c) for n, (c, _) in w.items()}, "left_tcp": r(extra["left_tcp"].pos),
                   "right_tcp": r(extra["right_tcp"].pos)}
            with (trace_dir / f"trace-{self.trace_index}.jsonl").open("a") as f:
                f.write(json.dumps(row) + "\n")
        return result

    def reset(self, scene, *, seed=None):
        self.task.module.reset_trial()
        self.trace_index = getattr(self, "trace_index", 0) + 1
        return original_reset(self, scene, seed=seed)

    cls._ensure_sim, cls._task_step, cls.reset = ensure, step, reset


if __name__ == "__main__":
    log_dir = None
    if "--log-dir" in sys.argv:
        log_dir = Path(sys.argv[sys.argv.index("--log-dir") + 1]).resolve()
        log_dir.mkdir(parents=True, exist_ok=True)
    install(log_dir)
    from inspect_robots.cli import main

    main()

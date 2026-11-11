"""Run inspect-robots with the bike rack (rack.build: a frame and two hinged support arms) spawned in place of the
`rack` placeholder box. score() also sees the rack's pose and hinge angles and both tool centre points, and the
scorer is reset at every trial. The bike is a partkit Part (partkit.py handles it). Same command line as
inspect-robots. With --log-dir, a per-step trace goes to <log-dir>/trace-<n>.jsonl."""

import atexit
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import rack as K  # noqa: E402

NAME = "rack"


def _np(x):
    import numpy as np

    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64)


def _rack_object(spec):
    """An Arena articulation whose spawner authors rack.build(); hinges get damping-only implicit actuators."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        stage = get_current_stage()
        root = UsdGeom.Xform.Define(stage, prim_path)
        root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
        K.build(stage, prim_path)
        return root.GetPrim()

    @configclass
    class RackSpawnerCfg(SpawnerCfg):
        func: object = clone(spawn)

    class RackObject(Object):
        def _generate_articulation_cfg(self):
            hinges = ImplicitActuatorCfg(joint_names_expr=list(K.JOINTS.values()), stiffness=0.0, damping=K.ARM_DAMPING)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"hinges": hinges})
            return self._add_initial_pose_to_cfg(cfg)

    return RackObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=RackSpawnerCfg(),
                      initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def install(trace_dir=None):
    import numpy as np
    from yam_arena import inspect_embodiment as arena
    from yam_arena.task_file import ObjectState

    from partkit import PartState

    cls = arena.YamArenaEmbodiment
    original_ensure, original_step, original_reset = cls._ensure_sim, cls._task_step, cls.reset
    patched = {"done": False}

    def install_scene():
        from yam_arena import scene

        original = scene.task_object

        def task_object(spec):
            return _rack_object(spec) if spec.name == NAME else original(spec)

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
        origin = _np(base.scene.env_origins[0])
        asset = base.scene[NAME]
        pose = _np(asset.data.root_link_pose_w)[0]
        vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        hinges = {n: K.SIGN * float(q[names.index(j)]) for n, j in K.JOINTS.items()}
        p = pose[:3] - origin
        extra = {NAME: PartState(pos=p, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=p, high=p, sliders=hinges)}
        for arm in arena.ARMS:
            t = np.asarray(tcp_world(base.scene[arm]), float) - origin
            z = np.zeros(3)
            extra[arm.split("_")[0] + "_tcp"] = ObjectState(pos=t, quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)), lin_vel=z,
                                                            ang_vel=z, low=t, high=t)
        original_states = scene.object_states

        def object_states(env, wanted):
            return {**original_states(env, [n for n in wanted if n not in extra]), **extra}

        scene.object_states = object_states
        try:
            result = original_step(self)
        finally:
            scene.object_states = original_states
        m = self.task.module
        if trace_dir is not None:
            import scoring

            states = {**original_states(self._env, ["bike"]), **extra}
            r = lambda v: [round(float(a), 4) for a in v]  # noqa: E731
            pos = scoring.position(states)
            row = {"time_s": round(self._t, 3), "score": result.info.get("score"), "fault": m.fault(),
                   "position": pos, "bike": r(states["bike"].pos), "lean_deg": round(math.degrees(scoring.lean(states)), 2),
                   "front": r(scoring.wheel(states, "front")), "rear": r(scoring.wheel(states, "rear")),
                   "hinge_deg": {n: round(math.degrees(v), 2) for n, v in hinges.items()},
                   "hook": {n: r(scoring.hook_world(states, n)) for n in hinges},
                   "hook_check": scoring.hook_on_front_tyre(states, "outer"), "rack": r(p),
                   "left_tcp": r(extra["left_tcp"].pos), "right_tcp": r(extra["right_tcp"].pos)}
            with (trace_dir / f"trace-{self.trace_index}.jsonl").open("a") as f:
                f.write(json.dumps(row) + "\n")
        return result

    def stow(self):
        """Swing both support arms back to stowed (hinge angle 0, at rest): the env's reset doesn't reset them."""
        base = getattr(self, "_base", None)
        if base is None:
            return
        import torch

        asset = base.scene[NAME]
        q = getattr(asset.data.joint_pos, "torch", asset.data.joint_pos)
        zero = torch.zeros_like(q)
        if hasattr(asset, "write_joint_state_to_sim_index"):
            asset.write_joint_state_to_sim_index(position=zero, velocity=zero)
        else:
            asset.write_joint_state_to_sim(zero, zero)

    def reset(self, scene, *, seed=None):
        self.task.module.reset_trial()
        self.trace_index = getattr(self, "trace_index", 0) + 1
        stow(self)
        obs = original_reset(self, scene, seed=seed)
        stow(self)
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = ensure, step, reset


if __name__ == "__main__":
    log_dir = None
    if "--log-dir" in sys.argv:
        log_dir = Path(sys.argv[sys.argv.index("--log-dir") + 1]).resolve()
        log_dir.mkdir(parents=True, exist_ok=True)
    install(log_dir)
    from inspect_robots.cli import main

    main()

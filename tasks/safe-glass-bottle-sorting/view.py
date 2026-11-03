"""Keep an interactive Isaac Sim window open; optionally replay the demonstration."""

import argparse
import time
from pathlib import Path

from inspect_robots import Scene
from inspect_robots.types import Action
from run import install
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.scripted import YamScriptedPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--demo", action="store_true", help="replay the scripted swap on a loop"
    )
    args = parser.parse_args()
    install()
    env = YamArenaEmbodiment(
        task=str(Path(__file__).with_name("task.py")),
        object_jitter=0,
        control_hz=20,
        cam_width=640,
        cam_height=360,
        headless=False,
    )
    scene = Scene(
        id="bottle-sorting-preview",
        instruction="Sort all bottles onto their matching colored shelves safely.",
    )
    policy = YamScriptedPolicy()
    policy.bind(env.info)
    obs = env.reset(scene, seed=0)
    policy.reset(scene)
    ready = Action(data=[*READY, 1.0, *READY, 1.0])
    import yam_arena.inspect_embodiment as ie

    try:
        while ie._APP.is_running():
            start = time.monotonic()
            action = policy.act(obs).actions[0] if args.demo else ready
            result = env.step(action)
            obs = result.observation
            if args.demo and (result.terminated or obs.state_time > 65):
                print(
                    "Preview episode:",
                    result.termination_reason,
                    result.info,
                    flush=True,
                )
                # Pause with rendering active, then restore the exact fixed layout.
                until = time.monotonic() + 3
                while time.monotonic() < until and ie._APP.is_running():
                    ie._APP.update()
                    time.sleep(0.02)
                obs = env.reset(scene, seed=0)
                policy.reset(scene)
            time.sleep(max(0, 0.05 - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()

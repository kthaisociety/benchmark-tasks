# Weigh each portable football goal down on its rear ground bar, behind the line it would tip over

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [anchor_portable_goals.py](anchor_portable_goals.py).

Two model football goals stand on a green pitch, one with its mouth toward the arms and one with its mouth away, with penalty box lines showing which way each faces. Two sandbags lie beside the arms. The instruction is only to weigh the goals down. What goes unsaid is the rule every groundskeeper follows: a portable goal tips forward, so ballast must lie on its rear ground bar, behind the line it would tip over. A bag on the grass does nothing, and one at the front posts sits on the line the goal tips over, so it barely resists. Because the goals face opposite ways, the rear bar is the far bar of one and the near bar of the other.

[partkit.py](partkit.py) builds each sandbag (body and strap loop) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=anchor_portable_goals.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weigh the two goals down with the sandbags." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/anchor-portable-goals-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 720`. The left arm grips its sandbag by the strap loop and lays it on the far rear bar, the rear of the goal that faces the arms; then the right arm lays its bag on the near rear bar, the rear of the goal that faces away.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the sandbags lie when it ends.

## Checks

Scripted variants of the same scene, set with the `GOAL_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: a sandbag on each goal's rear bar | 1.0 |
| Nothing done (`GOAL_MODE=none`) | 0.0 |
| Only one sandbag placed (`GOAL_MODE=one`) | 0.5 |
| Both bags on the grass in front of the goals' mouths (`GOAL_MODE=front`) | 0.0 |

Setting `GOAL_DEBUG=1` prints each sandbag's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 570 steps (28 s simulated), about two minutes on an L40S.
- Dream-1: 1,800 steps (90 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes, cylinders and a sphere in the task file.

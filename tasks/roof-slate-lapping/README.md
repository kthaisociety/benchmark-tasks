# Slate both faces of a gable roof so every slate laps over the one below

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [roof_slate_lapping.py](roof_slate_lapping.py).

A model gable roof stands between the arms. One face falls toward the arms to a gutter at the near eaves, the other falls away to a gutter at the far eaves, and chalk lines mark two slate courses on each face. The instruction is only to lay the four slates, one on each course. What goes unsaid is the roofer's rule: rain runs from the ridge down to the gutters, so on each face the slate nearer the ridge must lie on top of the one below it, which means laying each face from its eaves upward. Because the faces fall in opposite directions, neither "nearest course first" nor "farthest course first" gets both faces right: each scores 0.5.

[partkit.py](partkit.py) builds the roof (sarking, rafters, posts, ridge tile and chalk lines, at 25 degrees) and the slates as rigid parts.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=roof_slate_lapping.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the four slates on the roof, one on each course." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/roof-slate-lapping-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 950`. The left arm lays the near face, eaves course first; then the right arm lays the far face, starting at the far eaves, so on both faces the upper slate laps over the lower one.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the slates lie when it ends.

## Checks

Scripted variants of the same scene, set with the `SLATE_NEAR` and `SLATE_FAR` environment variables (the order the courses are laid on each face, `0` at the eaves):

| Variant | Score |
|---|---|
| Demo: both faces from the eaves up (`01`, `01`) | 1.0 |
| Nearest course first (`01`, `10`): near face right, far face ridge first | 0.5 |
| Farthest course first (`10`, `01`): near face ridge first, far face right | 0.5 |
| Both faces from the ridge down (`10`, `10`) | 0.0 |
| Nothing done | 0.0 |

The demo was also run for two trials in one process; both scored 1.0. Setting `SLATE_DEBUG=1` prints each slate's pose and course every 40 steps.

## Runtime and seeds

- Demo: about 920 steps (46 s simulated), about three minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.

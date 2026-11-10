# Set mine ventilation stoppings so air sweeps each working face before returning

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [mine_ventilation.py](mine_ventilation.py).

A tabletop ventilation plan shows two room-and-pillar development districts. Each has an intake and a return entry joined by crosscuts through the pillar row, and they are driven to different depths with two and four crosscuts. The instruction is only to set the stoppings for the two working faces. What goes unsaid is how the air has to travel: up the intake, across at the last open crosscut and back down the return. Every crosscut outby the last one needs a stopping. Sealing the last one too stops the air, and a stopping left in an entry blocks it.

[partkit.py](partkit.py) builds each stopping as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mine_ventilation.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the stoppings for the two working faces." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/mine-ventilation-stoppings-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1000`. The left arm seals the north district's first crosscut, then the right arm seals the south district's first three, leaving the last crosscut of each district open and the spare stopping on the supply track.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the stoppings stand when it ends.

## Checks

Calling `score()` directly on exact stopping positions:

| Layout | Score |
|---|---|
| Demo layout: north crosscut 1, south crosscuts 1 to 3 | 1.0 |
| Nothing done | 0.0 |
| Every north crosscut sealed, so no air reaches the north face | 0.5 |
| Every south crosscut sealed | 0.5 |
| Only the crosscut nearest the fan sealed in each district | 0.5 |
| Spare stopping left standing in the north intake, or in the south return | 0.5 |
| South crosscut 2 left open | 0.5 |
| North district only | 0.5 |

The task file also has scripted versions of these layouts, set with the `MINE_MODE` environment variable (`none`, `all`, `nearest`, `entry`, `one`). Setting `MINE_DEBUG=1` prints each district's result and each stopping's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 880 steps (44 s simulated), about six minutes on an L40S.
- Dream-1: 1,500 steps (75 s simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes in the task file.

# Anchor a hand line at the road and run it up both flanks of a grass fire

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [grass_fire_hand_line.py](grass_fire_hand_line.py).

A wildland fire crew's sand table: a grass fire runs away from the robot with the wind, its narrow heel just past a gravel road and its wide, flaming head at the far end. Six raked-earth strips lie beside it. The instruction is only to lay the hand line on the sand table. Hand crews anchor their line on a secure barrier, here the road, and build it along both flanks close to the fire edge toward the head. A line across the head of a running fire puts the crew in its path, so it scores nothing.

[partkit.py](partkit.py) builds each strip as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=grass_fire_hand_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the hand line on the sand table." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/grass-fire-hand-line-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. Each arm anchors a strip on the road beside the heel, then links two more strips up its flank toward the head.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the strips lie when it ends.

## Checks

Calling `score()` directly on exact strip positions:

| Layout | Score |
|---|---|
| Demo layout: both flanks anchored and linked to the head | 1.0 |
| Nothing done | 0.0 |
| Last strip on each flank laid across the head | 0.0 |
| Strips wrapped round the head | 0.0 |
| One strip on the burnt area | 0.0 |
| Left flank without its anchor strip | 0.5 |
| Left flank stops after two strips | 0.75 |
| Left flank missing its middle strip | 0.75 |
| Left flank line 9 cm off the fire edge | 0.5 |
| Strips laid along the table axis rather than turned to the flank | 1.0 |

The task file also has scripted versions of some of these layouts, set with the `FIRE_MODE` environment variable (`none`, `head`, `unanchored`, `one`). Setting `FIRE_DEBUG=1` prints each flank's result and each strip's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 700 steps (35 s simulated).
- Dream-1: 1,200 steps (60 s simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes in the task file.

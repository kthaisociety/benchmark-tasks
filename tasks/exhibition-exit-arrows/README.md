# Set floor exit arrows at an exhibition hall's aisle junctions toward the nearest exit by walking route

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [exhibition_exit_arrows.py](exhibition_exit_arrows.py).

An exhibition hall floor before the doors open: booths divide it into a grid of aisles, three aisle segments are closed by booths built across them, and there are fire exits in the far wall and the right wall. Four junctions are marked for arrows. The instruction is only to set the exit arrows. Each arrow should point along the aisle that starts the shortest walking route to an exit. At three of the four junctions that is not the exit nearest in a straight line, because a closed aisle forces a detour.

[partkit.py](partkit.py) builds each arrow mat, with its green head, as one rigid part. The task file computes the correct arrows from the aisle graph with a shortest-path search, so they follow from the layout.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=exhibition_exit_arrows.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the exit arrows for the hall." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/exhibition-exit-arrows-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. The left arm sets arrows at the two left junctions near the steward table; the right arm then sets the two right junctions further in.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the arrows lie when it ends.

## Checks

Calling `score()` directly on exact arrow poses:

| Layout | Score |
|---|---|
| Demo layout | 1.0 |
| Nothing done | 0.0 |
| Each arrow toward the exit nearest in a straight line | 0.25 |
| Every arrow toward the far wall | 0.25 |
| Every arrow toward the right-wall exit | 0.5 |
| Every arrow to the left | 0.25 |
| Every arrow back toward the steward table | 0.0 |
| One arrow 4 cm off its junction | 0.75 |
| One arrow turned 25 degrees off its aisle | 1.0 |

The task file also has scripted versions of some of these layouts, set with the `EXIT_MODE` environment variable (`none`, `straight_line`, `all_far`). Setting `EXIT_DEBUG=1` prints each junction's result and each arrow's position and heading every 40 steps.

## Runtime and seeds

- Demo: about 920 steps (46 s simulated).
- Dream-1: 1,200 steps (60 s simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes in the task file.

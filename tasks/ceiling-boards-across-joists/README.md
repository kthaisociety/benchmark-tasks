# Lay plasterboard across the ceiling joists, turning each board so both ends land on a joist

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [ceiling_boards_across_joists.py](ceiling_boards_across_joists.py).

A ceiling bay is built upside down on the table so it can be boarded from above: three joists run away from the arms between two wall plates, and two plasterboards lie on dollies beside the arms, their long edges running the same way as the joists. The instruction is only to lay the boards on the joists. What goes unsaid is the dryliner's rule: a ceiling board goes up with its long edges at right angles to the joists and both ends landing on a joist, so every edge is supported. Laid the way they are lying, the boards would run along the joists with their edges unsupported, so each must be turned a quarter turn first.

[partkit.py](partkit.py) builds the boards as rigid parts.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=ceiling_boards_across_joists.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the two boards on the joists." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/ceiling-boards-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 800`. The left arm lifts its board, turns it a quarter turn in the air and lays it across the joists with both ends on the outer joists; then the right arm does the same with the other board, nearer the arms.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the boards lie when it ends.

## Checks

Scripted variants of the same scene, set with the `CEIL_MODE` and `CEIL_Y` environment variables:

| Variant | Score |
|---|---|
| Demo: both boards turned and laid across the joists | 1.0 |
| Both laid the way they were lying, along the joists (`CEIL_MODE=parallel`) | 0.0 |
| Only one board turned (`CEIL_MODE=one`) | 0.5 |
| Both turned but laid 5 cm off centre, so their ends fall between joists (`CEIL_Y=0.05`) | 0.0 |
| Nothing done (`CEIL_MODE=none`) | 0.0 |

The demo was also run for two trials in one process; both scored 1.0. Setting `CEIL_DEBUG=1` prints each board's position, heading and whether it counts as laid every 40 steps.

## Runtime and seeds

- Demo: about 700 steps (35 s simulated), about two minutes on an L40S.
- Dream-1: 1,800 steps (90 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.

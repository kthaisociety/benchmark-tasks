# Find the heavier block with a balance

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [find_heavier_block.py](find_heavier_block.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/find-heavier-block/find_heavier_block.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "One of the three grey blocks is heavier than the others. Use the balance to find it and put only that block in the bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/heavier-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`. It weighs the left and right cubes against each other with both arms at once; the plank stays level, so it bins the middle one.

The run prints `task_score` at the end. `score()` in the task file returns 0 unless two cubes were weighed against each other, then 1.0 if the bin holds only the heavy cube and 0.25 otherwise; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 379 steps (19 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

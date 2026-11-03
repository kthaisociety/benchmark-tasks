# Restock a shelf lane with older stock in front

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [restock_fifo.py](restock_fifo.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/restock-fifo/restock_fifo.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Restock the lane with the blue can. The red can is older stock and must end up at the front." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/restock-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if a can ever tilts past 45 degrees, 1.0 for the red can in the front half of the lane and the blue can behind it, 0.5 for only the red can moved forward; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 357 steps (18 s simulated), about a minute on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

# Unload a cantilevered plank without tipping it

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [plank_unload.py](plank_unload.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/plank-unload/plank_unload.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the glass bottle and the steel block in the tray without letting the plank tip." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plank-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 600`.

The run prints `task_score` at the end. `score()` in the task file returns 0 if the plank ever tilts past 10 degrees or the bottle past 45, otherwise 0.5 per item in the tray; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 435 steps (22 s simulated), about 1.5 minutes on an L40S including start-up.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

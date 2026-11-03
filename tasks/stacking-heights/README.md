# Stack spacer blocks to a marked height

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [stacking_heights.py](stacking_heights.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/stacking-heights/stacking_heights.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack blocks on the grey pad so the top of the stack is level with the orange band on the post." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/stacking-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 800`. The right arm sets down the 6 cm spacer, then the left arm adds the 3 cm and 2 cm spacers.

The run prints `task_score` at the end. `score()` in the task file measures the stack on the pad: 1.0 within 0.5 cm of 11 cm, 0.25 within 1.5 cm, otherwise 0; `success()` is a score of 1.0.

## Runtime and seeds

- Demo: 592 steps (30 s simulated), about 1.5 minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

# Leave a dinghy on the beach above the high-tide line

A task file for the `yam_arena` embodiment (Isaac Lab-Arena) on the station scene: [dinghy_above_tide.py](dinghy_above_tide.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/dinghy-above-tide-line/dinghy_above_tide.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Leave the dinghy on the beach for the night." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/dinghy-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 400`.

The run prints `task_score` at the end. `score()` returns 0 unless the dinghy rests upright on the beach; then 1.0 if it is wholly 3 cm or more beyond the strandline's landward edge, 0.5 if beyond it by less.

## Runtime and seeds

- Demo: 191 steps, about 2 minutes on an L40S including start-up.
- Dream-1: 600 steps (30 seconds simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence

Task code under MIT. No third-party assets.
